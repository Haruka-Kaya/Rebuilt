package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

/** Pure release gate shared by teleoperated and autonomous shooting commands. */
final class ShotReleaseInterlock {
  enum Reason {
    READY,
    SHOOTER_NOT_READY,
    CONVEYOR_NOT_READY,
    FEEDER_NOT_READY,
    HUB_INACTIVE,
    AIM_NOT_READY
  }

  record Decision(boolean allowed, List<Reason> blockers) {
    Decision {
      blockers = List.copyOf(blockers);
      if (allowed != blockers.isEmpty()) {
        throw new IllegalArgumentException("allowed must match an empty blocker list");
      }
    }

    /** Compatibility accessor for callers that only need the first, highest-priority reason. */
    Reason reason() {
      return allowed ? Reason.READY : blockers.get(0);
    }

    String summary() {
      return allowed
          ? Reason.READY.name()
          : blockers.stream().map(Reason::name).collect(Collectors.joining("+"));
    }
  }

  private ShotReleaseInterlock() {}

  static boolean mayRelease(
      boolean shooterReady,
      boolean conveyorReady,
      boolean feederReady,
      boolean hubActive,
      boolean aimReady) {
    return evaluate(shooterReady, conveyorReady, feederReady, hubActive, aimReady).allowed();
  }

  static Decision evaluate(
      boolean shooterReady,
      boolean conveyorReady,
      boolean feederReady,
      boolean hubActive,
      boolean aimReady) {
    List<Reason> blockers = new ArrayList<>();
    if (!shooterReady) {
      blockers.add(Reason.SHOOTER_NOT_READY);
    }
    if (!conveyorReady) {
      blockers.add(Reason.CONVEYOR_NOT_READY);
    }
    if (!feederReady) {
      blockers.add(Reason.FEEDER_NOT_READY);
    }
    if (!hubActive) {
      blockers.add(Reason.HUB_INACTIVE);
    }
    if (!aimReady) {
      blockers.add(Reason.AIM_NOT_READY);
    }
    return new Decision(blockers.isEmpty(), blockers);
  }
}
