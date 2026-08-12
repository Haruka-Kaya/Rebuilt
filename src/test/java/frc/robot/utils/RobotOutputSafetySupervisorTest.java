package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
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
    assertEquals(RobotOutputSafetySupervisor.Phase.ARMED, supervisor.snapshot().phase());
    assertTrue(ProcessOutputSafety.isOutputAuthorized());
  }

  @Test
  void expiredHeartbeatRevokesStopsAndCannotRearmUntilDisabledStopIsConfirmed() {
    supervisor = createSupervisor();
    supervisor.serviceOnceForTesting();
    stopSessions.get(0).confirmed = true;
    supervisor.serviceOnceForTesting();
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
  void revokeWaitsForAnAuthorizedVendorCallAndBlocksEveryLaterCall() throws Exception {
    long generation = ProcessOutputSafety.revoke("TEST_PREPARE");
    assertTrue(ProcessOutputSafety.authorize(generation));
    CountDownLatch vendorEntered = new CountDownLatch(1);
    CountDownLatch releaseVendor = new CountDownLatch(1);
    ExecutorService executor = Executors.newFixedThreadPool(2);
    try {
      Future<ProcessOutputSafety.AuthorizedCall<Boolean>> vendorCall = executor.submit(
          () -> ProcessOutputSafety.callIfAuthorized(() -> {
            vendorEntered.countDown();
            try {
              assertTrue(releaseVendor.await(2, TimeUnit.SECONDS));
            } catch (InterruptedException exception) {
              Thread.currentThread().interrupt();
              throw new IllegalStateException(exception);
            }
            return Boolean.TRUE;
          }));
      assertTrue(vendorEntered.await(2, TimeUnit.SECONDS));
      Future<Long> revoke = executor.submit(
          () -> ProcessOutputSafety.revoke("TEST_REVOKED"));
      Thread.sleep(20L);
      assertFalse(revoke.isDone(), "revoke returned before the in-flight vendor call completed");
      releaseVendor.countDown();
      assertTrue(vendorCall.get(2, TimeUnit.SECONDS).authorized());
      revoke.get(2, TimeUnit.SECONDS);
      assertFalse(ProcessOutputSafety.isOutputAuthorized());
      assertFalse(ProcessOutputSafety.callIfAuthorized(() -> Boolean.TRUE).authorized());
    } finally {
      releaseVendor.countDown();
      executor.shutdownNow();
      assertTrue(executor.awaitTermination(2, TimeUnit.SECONDS));
    }
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
