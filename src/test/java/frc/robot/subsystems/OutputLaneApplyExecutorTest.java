package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.StatusCode;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class OutputLaneApplyExecutorTest {
  @Test
  void expiredPulseAtApplySkipsDelegateAndRetainsNeutralBarrier() {
    OutputLaneBarrierTracker lane = new OutputLaneBarrierTracker();
    AtomicBoolean pulseValid = new AtomicBoolean(false);
    AtomicInteger delegateCalls = new AtomicInteger();

    StatusCode result = OutputLaneApplyExecutor.apply(
        () -> OutputLaneReservation.reserveNonzero(
            lane, true, pulseValid.get(), true, 0L, 0L),
        () -> {
          delegateCalls.incrementAndGet();
          return StatusCode.OK;
        },
        lane::completeNonzero,
        () -> lane.reserveBarrier(4L));

    assertEquals(StatusCode.GeneralError, result);
    assertEquals(0, delegateCalls.get(), "expired pulse must not reach module.apply");
    assertTrue(lane.neutralRequired(), "rejected apply must retain a neutral barrier");
  }

  @Test
  void partialApplyFailureCompletesCutoffBeforeReservingCertifyingNeutral() {
    OutputLaneBarrierTracker lane = new OutputLaneBarrierTracker();
    AtomicInteger delegateCalls = new AtomicInteger();

    StatusCode result = OutputLaneApplyExecutor.apply(
        lane::beginNonzero,
        () -> {
          delegateCalls.incrementAndGet();
          throw new IllegalStateException("partial vendor apply");
        },
        lane::completeNonzero,
        () -> lane.reserveBarrier(9L));

    assertEquals(StatusCode.GeneralError, result);
    assertEquals(1, delegateCalls.get());
    assertTrue(lane.neutralRequired());
    assertTrue(!lane.hasPreBarrierInFlight(),
        "certifying neutral is reserved only after the ambiguous apply returns");
  }
}
