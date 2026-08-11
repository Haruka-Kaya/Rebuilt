package frc.robot.utils;

import java.util.function.Supplier;

import com.revrobotics.REVLibError;

/** Normalizes synchronous REV calls so vendor exceptions cannot escape the robot main loop. */
final class SparkVendorCall {
  private static final Result SUCCESS = new Result(REVLibError.kOk, null);

  record Result(REVLibError error, String failure) {
    boolean succeeded() {
      return error == REVLibError.kOk && failure == null;
    }
  }

  private SparkVendorCall() {}

  static Result execute(String operation, Supplier<REVLibError> action) {
    try {
      REVLibError error = action.get();
      if (error == null) {
        return new Result(REVLibError.kError, operation + " returned null");
      }
      return error == REVLibError.kOk ? SUCCESS : new Result(error, operation + " " + error);
    } catch (RuntimeException exception) {
      return new Result(
          REVLibError.kError,
          operation + " exception " + exception.getClass().getSimpleName());
    }
  }
}
