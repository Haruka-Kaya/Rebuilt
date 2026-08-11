package frc.robot.utils;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

/** Single-flight daemon worker used to keep blocking vendor I/O off the robot main thread. */
final class SparkAsyncWorker {
  private final ExecutorService executor;
  private final AtomicBoolean busy = new AtomicBoolean();

  private SparkAsyncWorker(ExecutorService executor) {
    this.executor = executor;
  }

  static SparkAsyncWorker createDaemon(String threadName) {
    return new SparkAsyncWorker(Executors.newSingleThreadExecutor(runnable -> {
      Thread thread = new Thread(runnable, threadName);
      thread.setDaemon(true);
      return thread;
    }));
  }

  static SparkAsyncWorker forTest(ExecutorService executor) {
    return new SparkAsyncWorker(executor);
  }

  boolean isIdle() {
    return !busy.get();
  }

  boolean submit(Runnable operation) {
    if (!busy.compareAndSet(false, true)) {
      return false;
    }
    try {
      executor.execute(() -> {
        try {
          operation.run();
        } finally {
          busy.set(false);
        }
      });
      return true;
    } catch (RejectedExecutionException exception) {
      busy.set(false);
      return false;
    }
  }

  /** Runs a finite batch of available safety follow-up work before releasing the single worker. */
  boolean submitDraining(
      Runnable operation, BooleanSupplier drainOne, int maximumDrainItems) {
    if (maximumDrainItems < 0) {
      throw new IllegalArgumentException("maximumDrainItems must be nonnegative");
    }
    return submit(() -> {
      try {
        operation.run();
      } finally {
        for (int drained = 0;
            drained < maximumDrainItems && drainOne.getAsBoolean();
            drained++) {
          // Each call performs one item. The finite batch keeps retries from starving health work.
        }
      }
    });
  }
}
