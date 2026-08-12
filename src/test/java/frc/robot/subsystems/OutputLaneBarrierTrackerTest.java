package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class OutputLaneBarrierTrackerTest {
  @Test
  void certifyingNeutralCannotCompleteBeforeEveryCutoffTicketReturns() {
    OutputLaneBarrierTracker lane = new OutputLaneBarrierTracker();
    long first = lane.beginNonzero();
    long second = lane.beginNonzero();
    var barrier = lane.reserveBarrier(7L);

    assertTrue(lane.hasPreBarrierInFlight());
    assertFalse(lane.markNeutralCompleted(barrier.generation(), barrier.outputEpoch()));
    assertFalse(lane.completeNonzero(second), "first cutoff call still owns ordering");
    assertTrue(lane.completeNonzero(first), "last cutoff completion releases certifying neutral");
    assertTrue(lane.markNeutralCompleted(barrier.generation(), barrier.outputEpoch()));
    assertFalse(lane.neutralRequired());
  }

  @Test
  void lateCompletionInvalidatesEarlyNeutralAndRetainsBarrier() {
    OutputLaneBarrierTracker lane = new OutputLaneBarrierTracker();
    long nonzero = lane.beginNonzero();
    var barrier = lane.reserveBarrier(11L);

    assertFalse(lane.markNeutralCompleted(barrier.generation(), barrier.outputEpoch()),
        "an overlapping neutral must never certify the barrier");
    assertTrue(lane.completeNonzero(nonzero));
    assertTrue(lane.neutralRequired(), "late nonzero completion cannot erase zero requirement");
    assertTrue(lane.markNeutralCompleted(barrier.generation(), barrier.outputEpoch()));
  }

  @Test
  void newerOutputEpochSupersedesOldStopEvidence() {
    OutputLaneBarrierTracker lane = new OutputLaneBarrierTracker();
    var oldBarrier = lane.reserveBarrier(2L);
    assertTrue(lane.markNeutralCompleted(oldBarrier.generation(), oldBarrier.outputEpoch()));

    lane.clearBarrierForNewOutputEpoch();
    lane.beginNonzero();

    assertFalse(lane.matches(oldBarrier.generation(), oldBarrier.outputEpoch()));
    assertFalse(lane.markNeutralCompleted(oldBarrier.generation(), oldBarrier.outputEpoch()));
  }

  @Test
  void repeatedBarrierReservationIsIdempotent() {
    OutputLaneBarrierTracker lane = new OutputLaneBarrierTracker();
    var first = lane.reserveBarrier(5L);
    var repeated = lane.reserveBarrier(5L);

    assertTrue(first.equals(repeated));
    assertTrue(lane.neutralRequired());
  }

  @Test
  void pendingBarrierRejectsNewEpochButCompletedBarrierMayBeSuperseded() {
    OutputLaneBarrierTracker lane = new OutputLaneBarrierTracker();
    var barrier = lane.reserveBarrier(8L);

    assertFalse(lane.canStartNewOutput(), "pending zero must remain ahead of later output");
    assertTrue(lane.matches(barrier.generation(), barrier.outputEpoch()));

    assertTrue(lane.markNeutralCompleted(barrier.generation(), barrier.outputEpoch()));
    assertTrue(lane.canStartNewOutput());
    lane.clearBarrierForNewOutputEpoch();
    assertFalse(lane.matches(barrier.generation(), barrier.outputEpoch()));
  }

  @Test
  void certifyingNeutralStartsAfterEveryCutoffVendorCallReturns() {
    OutputLaneBarrierTracker lane = new OutputLaneBarrierTracker();
    long first = lane.beginNonzero();
    long second = lane.beginNonzero();
    lane.reserveBarrier(12L);
    List<String> callLog = new ArrayList<>();

    callLog.add("N2-return");
    if (lane.completeNonzero(second)) {
      callLog.add("zero-start");
    }
    callLog.add("N1-return");
    if (lane.completeNonzero(first)) {
      callLog.add("zero-start");
    }

    assertTrue(callLog.equals(List.of("N2-return", "N1-return", "zero-start")));
  }

  @Test
  void retryReopensCompletedBarrierWithoutInvalidatingItsToken() {
    OutputLaneBarrierTracker lane = new OutputLaneBarrierTracker();
    var barrier = lane.reserveBarrier(21L);
    assertTrue(lane.markNeutralCompleted(barrier.generation(), barrier.outputEpoch()));

    assertTrue(lane.requireNeutralRetry(barrier.generation(), barrier.outputEpoch()));
    assertTrue(lane.matches(barrier.generation(), barrier.outputEpoch()));
    assertTrue(lane.neutralRequired());
  }

  @Test
  void staleNeutralCompletionCannotSatisfyANewerBarrier() {
    OutputLaneBarrierTracker lane = new OutputLaneBarrierTracker();
    var first = lane.reserveBarrier(30L);
    assertTrue(lane.markNeutralCompleted(first.generation(), first.outputEpoch()));
    lane.clearBarrierForNewOutputEpoch();

    long newerOutput = lane.beginNonzero();
    assertFalse(lane.completeNonzero(Long.MAX_VALUE), "unknown completion changes nothing");
    assertFalse(lane.completeNonzero(Long.MAX_VALUE), "duplicate unknown completion stays inert");
    assertFalse(lane.completeNonzero(newerOutput));
    assertFalse(lane.completeNonzero(newerOutput), "duplicate real completion stays inert");
    var second = lane.reserveBarrier(31L);

    assertFalse(lane.markNeutralCompleted(first.generation(), first.outputEpoch()));
    assertTrue(lane.neutralRequired());
    assertTrue(lane.markNeutralCompleted(second.generation(), second.outputEpoch()));
    assertFalse(lane.neutralRequired());
  }
}
