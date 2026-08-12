package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class OutputLaneLifecycleTrackerTest {
  @Test
  void closeIsLatchedAndNativeDestroyStartsExactlyOnceAfterDrainAndNeutral() {
    OutputLaneLifecycleTracker lifecycle = new OutputLaneLifecycleTracker();
    OutputLaneBarrierTracker lane = new OutputLaneBarrierTracker();
    long blockedNonzero = lane.beginNonzero();
    var closeBarrier = lane.reserveBarrier(4L);

    assertTrue(lifecycle.requestClose());
    assertFalse(lifecycle.requestClose(), "second close must not start another teardown");
    assertFalse(lifecycle.tryBeginNativeClose(
        !lane.hasInFlightNonzero(), !lane.neutralRequired()),
        "in-flight JNI must defer destroy");
    assertTrue(lane.completeNonzero(blockedNonzero));
    assertFalse(lifecycle.tryBeginNativeClose(
        !lane.hasInFlightNonzero(), !lane.neutralRequired()),
        "neutral barrier must complete first");
    assertTrue(lane.markNeutralCompleted(
        closeBarrier.generation(), closeBarrier.outputEpoch()));
    assertTrue(lifecycle.tryBeginNativeClose(true, true));
    assertFalse(lifecycle.tryBeginNativeClose(true, true));
    assertEquals(OutputLaneLifecycleTracker.Phase.CLOSED, lifecycle.phase());
    assertEquals(1, lifecycle.nativeCloseStarts());
  }
}
