package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;

class RuntimeSafetyLatchTest {
  @Test
  void startsHealthy() {
    RuntimeSafetyLatch latch = new RuntimeSafetyLatch();

    assertTrue(latch.healthy());
    assertEquals("HEALTHY", latch.reason());
    assertEquals(new RuntimeSafetyLatch.Snapshot(true, "HEALTHY"), latch.snapshot());
  }

  @Test
  void firstRuntimeExceptionFaultsTheLatchAndNormalizesItsReason() {
    RuntimeSafetyLatch latch = new RuntimeSafetyLatch();

    RuntimeSafetyLatch.Snapshot fault =
        latch.latch(new IllegalStateException("  scheduler\nfailed  "));

    assertFalse(fault.healthy());
    assertEquals("IllegalStateException: scheduler failed", fault.reason());
    assertEquals(fault, latch.snapshot());
  }

  @Test
  void nullAndBlankMessagesStillProduceNonblankReasons() {
    RuntimeSafetyLatch nullMessage = new RuntimeSafetyLatch();
    RuntimeSafetyLatch blankMessage = new RuntimeSafetyLatch();
    RuntimeSafetyLatch nullException = new RuntimeSafetyLatch();

    assertEquals("RuntimeException", nullMessage.latch(new RuntimeException()).reason());
    assertEquals(
        "IllegalArgumentException",
        blankMessage.latch(new IllegalArgumentException("  \t\n ")).reason());
    assertEquals("RuntimeException", nullException.latch(null).reason());
  }

  @Test
  void laterFaultsCannotResetOrReplaceTheFirstFault() {
    RuntimeSafetyLatch latch = new RuntimeSafetyLatch();
    RuntimeSafetyLatch.Snapshot first = latch.latch(new RuntimeException("first"));

    RuntimeSafetyLatch.Snapshot later = latch.latch(new RuntimeException("later"));

    assertEquals(first, later);
    assertEquals("RuntimeException: first", latch.reason());
    assertFalse(latch.healthy());
  }

  @Test
  void concurrentFaultsLatchExactlyOneCompleteReason() throws Exception {
    RuntimeSafetyLatch latch = new RuntimeSafetyLatch();
    int workers = 8;
    ExecutorService executor = Executors.newFixedThreadPool(workers);
    CountDownLatch ready = new CountDownLatch(workers);
    CountDownLatch start = new CountDownLatch(1);
    List<Future<?>> futures = new ArrayList<>();
    try {
      for (int index = 0; index < workers; index++) {
        int faultIndex = index;
        futures.add(executor.submit(() -> {
          ready.countDown();
          assertTrue(start.await(5, TimeUnit.SECONDS));
          latch.latch(new RuntimeException("fault-" + faultIndex));
          return null;
        }));
      }

      assertTrue(ready.await(5, TimeUnit.SECONDS));
      start.countDown();
      for (Future<?> future : futures) {
        future.get(5, TimeUnit.SECONDS);
      }
    } finally {
      executor.shutdownNow();
    }

    assertFalse(latch.healthy());
    assertTrue(latch.reason().matches("RuntimeException: fault-[0-7]"));
  }
}
