package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertTimeout;

import java.time.Duration;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import org.junit.jupiter.api.Test;

class SparkAsyncWorkerTest {
  @Test
  void submissionReturnsImmediatelyAndOnlyOneOperationRuns() throws Exception {
    var executor = Executors.newSingleThreadExecutor();
    try {
      SparkAsyncWorker worker = SparkAsyncWorker.forTest(executor);
      CountDownLatch started = new CountDownLatch(1);
      CountDownLatch release = new CountDownLatch(1);
      CountDownLatch finished = new CountDownLatch(1);

      assertTimeout(Duration.ofMillis(50), () -> assertTrue(worker.submit(() -> {
        started.countDown();
        try {
          release.await();
        } catch (InterruptedException exception) {
          Thread.currentThread().interrupt();
        } finally {
          finished.countDown();
        }
      })));

      assertTrue(started.await(1, TimeUnit.SECONDS));
      assertFalse(worker.isIdle());
      assertFalse(worker.submit(() -> {}));

      release.countDown();
      assertTrue(finished.await(1, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }
  }

  @Test
  void executorRejectionDoesNotLeaveWorkerBusy() {
    var executor = Executors.newSingleThreadExecutor();
    executor.shutdownNow();
    SparkAsyncWorker worker = SparkAsyncWorker.forTest(executor);

    assertFalse(worker.submit(() -> {}));
    assertTrue(worker.isIdle());
  }
}
