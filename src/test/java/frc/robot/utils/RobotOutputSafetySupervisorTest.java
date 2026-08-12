package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RobotOutputSafetySupervisorTest {
  private final AtomicReference<Double> now = new AtomicReference<>(1.0);
  private final AtomicBoolean enabled = new AtomicBoolean(false);
  private final List<FakeStopSession> stopSessions = new ArrayList<>();
  private RobotOutputSafetySupervisor supervisor;

  @BeforeEach
  void resetProcessAuthorization() {
    ProcessOutputSafety.resetForTesting();
  }

  @AfterEach
  void closeSupervisor() {
    if (supervisor != null) {
      supervisor.close();
    }
    ProcessOutputSafety.resetForTesting();
  }

  @Test
  void startupRequiresDisabledStopProofBeforeTheFirstEnabledHeartbeatCanAuthorize() {
    supervisor = createSupervisor();

    assertFalse(ProcessOutputSafety.isOutputAuthorized());
    supervisor.serviceOnceForTesting();
    assertEquals(1, stopSessions.size());
    assertEquals(RobotOutputSafetySupervisor.Phase.STARTUP_STOPPING,
        supervisor.snapshot().phase());

    stopSessions.get(0).confirmed = true;
    supervisor.serviceOnceForTesting();
    assertEquals(RobotOutputSafetySupervisor.Phase.READY_DISABLED,
        supervisor.snapshot().phase());
    assertFalse(ProcessOutputSafety.isOutputAuthorized());

    enabled.set(true);
    supervisor.heartbeat();
    assertEquals(RobotOutputSafetySupervisor.Phase.READY_DISABLED,
        supervisor.snapshot().phase());
    assertEquals("SCHEDULER_COMPLETION_NOT_OBSERVED", supervisor.snapshot().reason());
    assertFalse(ProcessOutputSafety.isOutputAuthorized());

    enabled.set(false);
    completeSchedulerRun();
    enabled.set(true);
    supervisor.heartbeat();
    assertEquals(RobotOutputSafetySupervisor.Phase.ARMED, supervisor.snapshot().phase());
    assertTrue(ProcessOutputSafety.isOutputAuthorized());
  }

  @Test
  void expiredHeartbeatRevokesStopsAndCannotRearmUntilDisabledStopIsConfirmed() {
    supervisor = createSupervisor();
    supervisor.serviceOnceForTesting();
    stopSessions.get(0).confirmed = true;
    supervisor.serviceOnceForTesting();
    completeSchedulerRun();
    enabled.set(true);
    supervisor.heartbeat();
    assertTrue(ProcessOutputSafety.isOutputAuthorized());

    now.set(1.101);
    supervisor.serviceOnceForTesting();
    assertFalse(ProcessOutputSafety.isOutputAuthorized());
    assertEquals(RobotOutputSafetySupervisor.Phase.STOPPING, supervisor.snapshot().phase());
    assertEquals(2, stopSessions.size());
    assertTrue(stopSessions.get(1).serviceCount > 0);

    supervisor.heartbeat();
    assertFalse(ProcessOutputSafety.isOutputAuthorized(),
        "a heartbeat in the same enabled session must not undo a safety trip");
    stopSessions.get(1).confirmed = true;
    supervisor.serviceOnceForTesting();
    assertEquals(RobotOutputSafetySupervisor.Phase.TRIPPED, supervisor.snapshot().phase());

    supervisor.heartbeat();
    assertFalse(ProcessOutputSafety.isOutputAuthorized());
    enabled.set(false);
    supervisor.serviceOnceForTesting();
    assertEquals(3, stopSessions.size());
    stopSessions.get(2).confirmed = true;
    supervisor.serviceOnceForTesting();
    assertEquals(RobotOutputSafetySupervisor.Phase.READY_DISABLED,
        supervisor.snapshot().phase());

    completeSchedulerRun();
    enabled.set(true);
    now.set(2.0);
    supervisor.heartbeat();
    assertTrue(ProcessOutputSafety.isOutputAuthorized());
  }

  @Test
  void stopFactoryAndEvidenceExceptionsRemainFailClosedAndRetryable() {
    AtomicBoolean throwFactory = new AtomicBoolean(true);
    supervisor = new RobotOutputSafetySupervisor(
        () -> now.get(),
        enabled::get,
        () -> {
          if (throwFactory.getAndSet(false)) {
            throw new IllegalStateException("synthetic stop request failure");
          }
          FakeStopSession session = new FakeStopSession();
          stopSessions.add(session);
          return session;
        },
        0.10,
        0.005,
        false);

    supervisor.serviceOnceForTesting();
    assertFalse(ProcessOutputSafety.isOutputAuthorized());
    assertTrue(supervisor.snapshot().stopSummary().contains("STOP_REQUEST_EXCEPTION"));
    supervisor.serviceOnceForTesting();
    assertEquals(1, stopSessions.size());
    stopSessions.get(0).throwService = true;
    supervisor.serviceOnceForTesting();
    assertFalse(ProcessOutputSafety.isOutputAuthorized());
    stopSessions.get(0).throwService = false;
    stopSessions.get(0).confirmed = true;
    supervisor.serviceOnceForTesting();
    assertEquals(RobotOutputSafetySupervisor.Phase.READY_DISABLED,
        supervisor.snapshot().phase());
  }

  @Test
  void irreversibleRuntimeFaultNeverPublishesAHealthyRearmStateAfterStopConfirmation() {
    supervisor = createSupervisor();
    supervisor.serviceOnceForTesting();
    stopSessions.get(0).confirmed = true;
    supervisor.serviceOnceForTesting();
    completeSchedulerRun();
    enabled.set(true);
    supervisor.heartbeat();

    supervisor.forceTrip("RUNTIME_FAULT_SYNTHETIC");
    assertFalse(ProcessOutputSafety.isOutputAuthorized());
    enabled.set(false);
    stopSessions.get(1).confirmed = true;
    supervisor.serviceOnceForTesting();
    supervisor.forceTrip("RUNTIME_FAULT_LATCHED");

    assertEquals(RobotOutputSafetySupervisor.Phase.TRIPPED, supervisor.snapshot().phase());
    assertTrue(supervisor.snapshot().irreversibleRuntimeFault());
    assertEquals("RUNTIME_FAULT_SYNTHETIC", supervisor.snapshot().schedulerFaultReason());
    int completedStopCount = stopSessions.size();
    supervisor.heartbeat();
    supervisor.serviceOnceForTesting();
    assertEquals(completedStopCount, stopSessions.size());
    assertFalse(ProcessOutputSafety.isOutputAuthorized());
  }

  @Test
  void heartbeatCannotRenewAuthorizationWithoutANewCompletedSchedulerEpoch() {
    supervisor = createSupervisor();
    supervisor.serviceOnceForTesting();
    stopSessions.get(0).confirmed = true;
    supervisor.serviceOnceForTesting();
    completeSchedulerRun();
    enabled.set(true);

    supervisor.heartbeat();
    assertEquals(RobotOutputSafetySupervisor.Phase.ARMED, supervisor.snapshot().phase());
    assertEquals(1.10, supervisor.snapshot().heartbeatDeadlineSeconds(), 1e-9);
    assertTrue(ProcessOutputSafety.isOutputAuthorized());

    // Starting the next scheduler pass is not completion evidence. Repeated heartbeat calls must
    // leave the original deadline unchanged until that exact epoch returns normally.
    long stalledSchedulerEpoch = supervisor.beginSchedulerRun();
    now.set(1.05);
    supervisor.heartbeat();
    now.set(1.09);
    supervisor.heartbeat();

    assertEquals(1.10, supervisor.snapshot().heartbeatDeadlineSeconds(), 1e-9);
    assertEquals("SCHEDULER_COMPLETION_NOT_OBSERVED", supervisor.snapshot().reason());
    assertEquals(
        "SCHEDULER_COMPLETION_NOT_OBSERVED", supervisor.snapshot().schedulerFaultReason());
    assertTrue(ProcessOutputSafety.isOutputAuthorized(),
        "the independent bounded deadline, not the caller, owns revocation timing");

    now.set(1.101);
    supervisor.serviceOnceForTesting();
    assertFalse(ProcessOutputSafety.isOutputAuthorized());
    assertEquals(RobotOutputSafetySupervisor.Phase.STOPPING, supervisor.snapshot().phase());

    // Completion after expiry is recorded for diagnostics but cannot re-arm the enabled session.
    supervisor.completeSchedulerRun(stalledSchedulerEpoch);
    supervisor.heartbeat();
    assertFalse(ProcessOutputSafety.isOutputAuthorized());
  }

  @Test
  void onlyTheMatchingNormallyCompletedSchedulerEpochCanRenew() {
    supervisor = createSupervisor();
    supervisor.serviceOnceForTesting();
    stopSessions.get(0).confirmed = true;
    supervisor.serviceOnceForTesting();
    completeSchedulerRun();
    enabled.set(true);
    supervisor.heartbeat();

    long nextEpoch = supervisor.beginSchedulerRun();
    supervisor.completeSchedulerRun(nextEpoch);
    now.set(1.05);
    supervisor.heartbeat();

    assertEquals(1.15, supervisor.snapshot().heartbeatDeadlineSeconds(), 1e-9);
    assertNull(supervisor.snapshot().schedulerFaultReason());
    assertTrue(ProcessOutputSafety.isOutputAuthorized());

    supervisor.completeSchedulerRun(nextEpoch);
    assertFalse(ProcessOutputSafety.isOutputAuthorized(),
        "a repeated or stale completion must fail closed");
    assertEquals(
        "SCHEDULER_COMPLETION_EPOCH_MISMATCH", supervisor.snapshot().schedulerFaultReason());
  }

  @Test
  void revokeRemainsBoundedAfterAClaimedTransactionAndRejectsEveryLaterPermit()
      throws Exception {
    long generation = ProcessOutputSafety.revoke("TEST_PREPARE");
    assertTrue(ProcessOutputSafety.authorize(generation));
    ProcessOutputSafety.NonzeroPermit admitted =
        ProcessOutputSafety.acquireNonzeroPermit().orElseThrow();
    assertTrue(ProcessOutputSafety.claimIfCurrent(admitted));
    assertFalse(ProcessOutputSafety.claimIfCurrent(admitted), "a permit must be one-shot");
    CountDownLatch vendorEntered = new CountDownLatch(1);
    CountDownLatch releaseVendor = new CountDownLatch(1);
    ExecutorService executor = Executors.newFixedThreadPool(2);
    try {
      Future<Boolean> vendorCall = executor.submit(() -> {
        vendorEntered.countDown();
        try {
          assertTrue(releaseVendor.await(2, TimeUnit.SECONDS));
        } catch (InterruptedException exception) {
          Thread.currentThread().interrupt();
          throw new IllegalStateException(exception);
        }
        return Boolean.TRUE;
      });
      assertTrue(vendorEntered.await(2, TimeUnit.SECONDS));
      Future<Long> revoke = executor.submit(
          () -> ProcessOutputSafety.revoke("TEST_REVOKED"));
      assertTrue(revoke.get(200, TimeUnit.MILLISECONDS) > generation,
          "a stalled vendor API must not block process-wide revocation");
      assertFalse(ProcessOutputSafety.isOutputAuthorized());
      assertTrue(ProcessOutputSafety.acquireNonzeroPermit().isEmpty());
      releaseVendor.countDown();
      assertTrue(vendorCall.get(2, TimeUnit.SECONDS));
    } finally {
      releaseVendor.countDown();
      executor.shutdownNow();
      assertTrue(executor.awaitTermination(2, TimeUnit.SECONDS));
    }
  }

  @Test
  void aPermitCapturedBeforeRevokeCannotBeClaimedAfterward() {
    long generation = ProcessOutputSafety.revoke("TEST_PREPARE");
    assertTrue(ProcessOutputSafety.authorize(generation));
    ProcessOutputSafety.NonzeroPermit stale =
        ProcessOutputSafety.acquireNonzeroPermit().orElseThrow();

    ProcessOutputSafety.revoke("TEST_REVOKED");

    assertFalse(ProcessOutputSafety.claimIfCurrent(stale));
    assertFalse(ProcessOutputSafety.claimIfCurrent(stale), "a failed claim also consumes the permit");
  }

  @Test
  void aLateHeartbeatCannotReviveAnExpiredGrantEvenWithFreshSchedulerCompletion() {
    supervisor = createSupervisor();
    supervisor.serviceOnceForTesting();
    stopSessions.get(0).confirmed = true;
    supervisor.serviceOnceForTesting();
    completeSchedulerRun();
    enabled.set(true);
    supervisor.heartbeat();

    completeSchedulerRun();
    now.set(1.101);
    supervisor.heartbeat();

    assertFalse(ProcessOutputSafety.isOutputAuthorized());
    assertEquals(RobotOutputSafetySupervisor.Phase.STOPPING, supervisor.snapshot().phase());
    assertEquals("ROBOT_LOOP_HEARTBEAT_EXPIRED", supervisor.snapshot().reason());
  }

  @Test
  void aCompletionProducedBeforeTripCannotRearmAfterDisabledStopProof() {
    supervisor = createSupervisor();
    supervisor.serviceOnceForTesting();
    stopSessions.get(0).confirmed = true;
    supervisor.serviceOnceForTesting();
    completeSchedulerRun();
    enabled.set(true);
    supervisor.heartbeat();

    completeSchedulerRun();
    supervisor.forceTrip("ROBOT_LOOP_HEARTBEAT_EXPIRED");
    enabled.set(false);
    stopSessions.get(1).confirmed = true;
    supervisor.serviceOnceForTesting();
    enabled.set(true);
    supervisor.heartbeat();

    assertFalse(ProcessOutputSafety.isOutputAuthorized());
    assertEquals(RobotOutputSafetySupervisor.Phase.READY_DISABLED, supervisor.snapshot().phase());
    assertEquals("SCHEDULER_COMPLETION_NOT_OBSERVED", supervisor.snapshot().reason());

    completeSchedulerRun();
    supervisor.heartbeat();
    assertTrue(ProcessOutputSafety.isOutputAuthorized());
  }

  @Test
  void aSecondTripAlsoInvalidatesCompletionProducedDuringTheFirstStop() {
    supervisor = createSupervisor();
    supervisor.serviceOnceForTesting();
    stopSessions.get(0).confirmed = true;
    supervisor.serviceOnceForTesting();
    completeSchedulerRun();
    enabled.set(true);
    supervisor.heartbeat();

    supervisor.forceTrip("FIRST_TRIP");
    completeSchedulerRun();
    supervisor.forceTrip("SECOND_TRIP");
    enabled.set(false);
    stopSessions.get(1).confirmed = true;
    supervisor.serviceOnceForTesting();
    enabled.set(true);
    supervisor.heartbeat();

    assertFalse(ProcessOutputSafety.isOutputAuthorized());
    assertEquals(RobotOutputSafetySupervisor.Phase.READY_DISABLED, supervisor.snapshot().phase());
    assertEquals("SCHEDULER_COMPLETION_NOT_OBSERVED", supervisor.snapshot().reason());

    completeSchedulerRun();
    supervisor.heartbeat();
    assertTrue(ProcessOutputSafety.isOutputAuthorized());
  }

  private RobotOutputSafetySupervisor createSupervisor() {
    return new RobotOutputSafetySupervisor(
        () -> now.get(),
        enabled::get,
        () -> {
          FakeStopSession session = new FakeStopSession();
          stopSessions.add(session);
          return session;
        },
        0.10,
        0.005,
        false);
  }

  private void completeSchedulerRun() {
    long epoch = supervisor.beginSchedulerRun();
    supervisor.completeSchedulerRun(epoch);
  }

  private static final class FakeStopSession
      implements RobotOutputSafetySupervisor.StopSession {
    private int serviceCount;
    private boolean confirmed;
    private boolean throwService;

    @Override
    public void service() {
      serviceCount++;
      if (throwService) {
        throw new IllegalStateException("synthetic evidence failure");
      }
    }

    @Override
    public boolean confirmed() {
      return confirmed;
    }

    @Override
    public String summary() {
      return confirmed ? "CONFIRMED" : "PENDING";
    }
  }
}
