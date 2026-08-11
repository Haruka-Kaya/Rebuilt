package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertTimeout;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

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

  @Test
  void drainsEveryAvailableSafetyItemInTheSameWorkerTurn() throws Exception {
    var executor = Executors.newSingleThreadExecutor();
    try {
      SparkAsyncWorker worker = SparkAsyncWorker.forTest(executor);
      List<Integer> order = new ArrayList<>();
      AtomicInteger next = new AtomicInteger(1);
      CountDownLatch drained = new CountDownLatch(1);

      assertTrue(worker.submitDraining(
          () -> order.add(0),
          () -> {
            int item = next.getAndIncrement();
            if (item <= 3) {
              order.add(item);
              if (item == 3) {
                drained.countDown();
              }
              return true;
            }
            order.add(item);
            return true;
          },
          3));

      assertTrue(drained.await(1, TimeUnit.SECONDS));
      long idleDeadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(1);
      while (!worker.isIdle() && System.nanoTime() < idleDeadline) {
        Thread.onSpinWait();
      }
      assertTrue(worker.isIdle());
      assertEquals(List.of(0, 1, 2, 3), order);
      assertEquals(4, next.get(), "the finite batch must not retry a due item forever");
    } finally {
      executor.shutdownNow();
    }
  }

  @Test
  void pendingSafetyWorkRunsImmediatelyAfterABlockedPrimaryOperation() throws Exception {
    var executor = Executors.newSingleThreadExecutor();
    try {
      SparkAsyncWorker worker = SparkAsyncWorker.forTest(executor);
      CountDownLatch primaryStarted = new CountDownLatch(1);
      CountDownLatch releasePrimary = new CountDownLatch(1);
      CountDownLatch safetyWorkRan = new CountDownLatch(1);
      CountDownLatch drainReachedEmpty = new CountDownLatch(1);
      CountDownLatch releaseDrain = new CountDownLatch(1);
      AtomicInteger pendingSafetyItems = new AtomicInteger();

      assertTrue(worker.submitDraining(
          () -> {
            primaryStarted.countDown();
            try {
              releasePrimary.await();
            } catch (InterruptedException exception) {
              Thread.currentThread().interrupt();
            }
          },
          () -> {
            if (pendingSafetyItems.getAndUpdate(value -> Math.max(0, value - 1)) > 0) {
              safetyWorkRan.countDown();
              return true;
            }
            drainReachedEmpty.countDown();
            try {
              releaseDrain.await();
            } catch (InterruptedException exception) {
              Thread.currentThread().interrupt();
            }
            return false;
          },
          10));

      assertTrue(primaryStarted.await(1, TimeUnit.SECONDS));
      pendingSafetyItems.incrementAndGet();
      releasePrimary.countDown();

      assertTrue(safetyWorkRan.await(1, TimeUnit.SECONDS));
      assertTrue(drainReachedEmpty.await(1, TimeUnit.SECONDS));
      assertFalse(worker.submit(() -> {}), "drain must finish before the worker becomes idle");
      releaseDrain.countDown();
    } finally {
      executor.shutdownNow();
    }
  }
}
