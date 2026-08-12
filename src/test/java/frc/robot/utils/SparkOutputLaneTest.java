package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SparkOutputLaneTest {
  @Test
  void stopDuringNonzeroVendorCallMustWaitForAPostCallZero() {
    SparkOutputLane lane = new SparkOutputLane();
    long nonzero = lane.reserveNonzero();

    lane.requestZero();
    assertFalse(lane.canReserveZero(), "zero cannot overtake admitted nonzero vendor JNI");
    assertFalse(lane.zeroCompletedFor(nonzero, nonzero - 1));

    assertTrue(lane.completeNonzero(nonzero));
    assertTrue(lane.canReserveZero());
    assertFalse(lane.zeroCompletedFor(nonzero, nonzero - 1));

    lane.zeroCompleted();
    assertTrue(lane.zeroCompletedFor(nonzero, nonzero));
  }

  @Test
  void passiveFollowerReservationRetainsAConcurrentGlobalStop() {
    SparkOutputLane leader = new SparkOutputLane();
    SparkOutputLane follower = new SparkOutputLane();
    long leaderReservation = leader.reserveNonzero();
    long followerReservation = follower.reserveNonzero();

    follower.requestZero();
    assertFalse(follower.canReserveZero(), "follower zero cannot pass leader JNI reservation");

    assertFalse(leader.completeNonzero(leaderReservation));
    assertTrue(follower.completeNonzero(followerReservation));
    assertFalse(follower.zeroCompletedFor(followerReservation, followerReservation - 1));
    assertTrue(follower.canReserveZero());
  }

  @Test
  void cleanupVisibilityRemainsInFlightUntilCompletion() {
    SparkOutputLane lane = new SparkOutputLane();
    long reservation = lane.reserveNonzero();

    assertTrue(lane.nonzeroInFlight());
    assertFalse(lane.reserveNonzero() >= 0L, "reservation is exclusive and retry-safe");

    lane.completeNonzero(reservation);
    assertFalse(lane.nonzeroInFlight());
    assertTrue(lane.reserveNonzero() >= 0L);
  }

  @Test
  void oldZeroCannotConfirmAStopWhileANewerNonzeroIsInFlight() {
    SparkOutputLane lane = new SparkOutputLane();
    long first = lane.reserveNonzero();
    lane.completeNonzero(first);
    lane.zeroCompleted();
    long requestedStop = lane.stopBarrierGeneration();
    assertTrue(lane.zeroCompletedFor(requestedStop, requestedStop));

    long newer = lane.reserveNonzero();
    assertFalse(lane.zeroCompletedFor(requestedStop, requestedStop));

    lane.requestZero();
    lane.completeNonzero(newer);
    assertFalse(lane.zeroCompletedFor(requestedStop, requestedStop));
    lane.zeroCompleted();
    assertTrue(lane.zeroCompletedFor(requestedStop, newer));
  }

  @Test
  void aStopRequestedWhileExternalAuthorizationIsEvaluatedInvalidatesThatEvaluation() {
    SparkOutputLane lane = new SparkOutputLane();
    long beforeAuthorization = lane.stopSequence();

    lane.requestZero();

    assertFalse(lane.stopSequence() == beforeAuthorization);
    assertTrue(lane.canReserveZero());
  }

  @Test
  void aDependentOnlyStopCanInvalidateItsLeadersPendingAuthorizationSnapshot() {
    SparkOutputLane leader = new SparkOutputLane();
    SparkOutputLane follower = new SparkOutputLane();
    long leaderAuthorization = leader.stopSequence();

    follower.requestZero();
    leader.requestZero();

    assertFalse(leader.stopSequence() == leaderAuthorization);
    assertTrue(follower.canReserveZero());
  }

  @Test
  void aStaleZeroCompletionCannotClearAStopBehindAnAdmittedNonzero() {
    SparkOutputLane lane = new SparkOutputLane();
    long nonzero = lane.reserveNonzero();
    lane.requestZero();

    lane.zeroCompleted();

    assertTrue(lane.nonzeroInFlight());
    assertTrue(lane.zeroPendingBehindNonzero());
    assertFalse(lane.zeroCompletedFor(nonzero, nonzero));
  }
}
