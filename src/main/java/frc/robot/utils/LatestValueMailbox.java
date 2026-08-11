package frc.robot.utils;

import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Thread-safe, latest-value-only mailbox for telemetry that must not block a producer thread.
 * Older unpublished values are intentionally replaced when the consumer is slower than the
 * producer.
 */
public final class LatestValueMailbox<T> {
  private final AtomicReference<T> latest = new AtomicReference<>();

  public void offer(T value) {
    latest.set(Objects.requireNonNull(value));
  }

  public Optional<T> takeLatest() {
    return Optional.ofNullable(latest.getAndSet(null));
  }
}
