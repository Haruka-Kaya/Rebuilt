package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.time.Duration;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.Test;

class OutputLaneReservationTest {
  @Test
  void blockingExternalAuthorizationCannotBlockNeutralBarrierReservation() throws Exception {
    OutputLaneBarrierTracker lane = new OutputLaneBarrierTracker();
    Object applicationLock = new Object();
    CountDownLatch authorizationEntered = new CountDownLatch(1);
    CountDownLatch releaseAuthorization = new CountDownLatch(1);
    AtomicLong admittedTicket = new AtomicLong(Long.MIN_VALUE);
    AtomicLong stopSequence = new AtomicLong();

    Thread admission = new Thread(() -> {
      boolean authorizationSnapshot;
      authorizationEntered.countDown();
      try {
        releaseAuthorization.await();
        authorizationSnapshot = true;
      } catch (InterruptedException exception) {
        Thread.currentThread().interrupt();
        authorizationSnapshot = false;
      }
      synchronized (applicationLock) {
        admittedTicket.set(OutputLaneReservation.reserveNonzero(
            lane, true, authorizationSnapshot, true, 0L, stopSequence.get()));
      }
    });
    admission.start();
    assertTrue(authorizationEntered.await(1, TimeUnit.SECONDS));

    try {
      assertTimeoutPreemptively(Duration.ofMillis(500), () -> {
        synchronized (applicationLock) {
          stopSequence.incrementAndGet();
          lane.reserveBarrier(3L);
        }
      });
    } finally {
      releaseAuthorization.countDown();
    }
    admission.join(1000L);
    assertEquals(-1L, admittedTicket.get(), "pending barrier must reject the stale snapshot");
  }

  @Test
  void stopSequenceRejectsAuthorizationSnapshotEvaluatedBeforeStop() {
    OutputLaneBarrierTracker lane = new OutputLaneBarrierTracker();

    assertEquals(-1L, OutputLaneReservation.reserveNonzero(
        lane, true, true, true, 14L, 15L));
    assertTrue(lane.canStartNewOutput());
  }
}
