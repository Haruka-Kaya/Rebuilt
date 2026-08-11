package frc.robot.utils;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.atomic.AtomicBoolean;

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
}
