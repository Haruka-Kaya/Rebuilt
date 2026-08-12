package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import java.util.function.LongConsumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

/** Executes one admitted CTRE module apply and retains neutral after every ambiguous outcome. */
final class OutputLaneApplyExecutor {
  private OutputLaneApplyExecutor() {}

  static StatusCode apply(
      LongSupplier admission,
      Supplier<StatusCode> vendorApply,
      LongConsumer completion,
      Runnable reserveNeutral) {
    long ticket = -1L;
    try {
      ticket = admission.getAsLong();
    } catch (RuntimeException ignored) {
      // Reservation exhaustion or another ambiguous admission failure is fail-closed below.
    }
    if (ticket < 0) {
      reserveNeutral.run();
      return StatusCode.GeneralError;
    }

    StatusCode result = StatusCode.GeneralError;
    try {
      result = vendorApply.get();
    } catch (RuntimeException ignored) {
      // A throwing vendor apply may have changed only some modules; neutral must follow it.
    } finally {
      try {
        completion.accept(ticket);
      } catch (RuntimeException ignored) {
        result = StatusCode.GeneralError;
      }
    }
    if (result == null || !result.isOK()) {
      reserveNeutral.run();
      return StatusCode.GeneralError;
    }
    return result;
  }
}
