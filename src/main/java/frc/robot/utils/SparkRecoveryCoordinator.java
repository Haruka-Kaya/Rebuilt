package frc.robot.utils;

import java.util.OptionalInt;
import java.util.function.IntPredicate;

/** Pure round-robin permit coordinator for expensive SPARK recovery operations. */
final class SparkRecoveryCoordinator {
  static final double CONFIG_OPERATION_INTERVAL_SECONDS = 0.20;

  private int cursor;
  private double nextConfigPermitAt;

  OptionalInt selectConfiguration(
      double nowSeconds,
      int deviceCount,
      boolean workerIdle,
      boolean configurationAllowed,
      IntPredicate deviceIsDue) {
    if (!workerIdle
        || !configurationAllowed
        || deviceCount <= 0
        || nowSeconds < nextConfigPermitAt) {
      return OptionalInt.empty();
    }

    for (int offset = 0; offset < deviceCount; offset++) {
      int index = (cursor + offset) % deviceCount;
      if (deviceIsDue.test(index)) {
        cursor = (index + 1) % deviceCount;
        nextConfigPermitAt = nowSeconds + CONFIG_OPERATION_INTERVAL_SECONDS;
        return OptionalInt.of(index);
      }
    }
    return OptionalInt.empty();
  }
}
