package frc.robot.utils;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.LockSupport;

/**
 * Bounded, nonblocking console sink. Slow Driver Station/stdout consumers can drop old diagnostics
 * but can never hold the robot main loop on {@code PrintStream}'s lock.
 */
public final class AsyncDiagnosticSink {
  private static final AtomicReference<String> PENDING = new AtomicReference<>();
  private static final AtomicLong DROPPED = new AtomicLong();
  private static final Thread WORKER;

  static {
    WORKER = new Thread(AsyncDiagnosticSink::drain, "robot-diagnostic-output");
    WORKER.setDaemon(true);
    WORKER.start();
  }

  private AsyncDiagnosticSink() {}

  public static void log(String message) {
    String safeMessage = Objects.requireNonNullElse(message, "null diagnostic");
    if (PENDING.getAndSet(safeMessage) != null) {
      DROPPED.incrementAndGet();
    }
    LockSupport.unpark(WORKER);
  }

  private static void drain() {
    while (!Thread.currentThread().isInterrupted()) {
      try {
        String message = PENDING.getAndSet(null);
        if (message == null) {
          LockSupport.park();
          continue;
        }
        long dropped = DROPPED.getAndSet(0L);
        if (dropped > 0) {
          System.out.println("ASYNC_DIAGNOSTICS dropped=" + dropped);
        }
        System.out.println(message);
      } catch (RuntimeException exception) {
        // Keep diagnostics best-effort. Motor control must not depend on console availability.
      }
    }
  }
}
