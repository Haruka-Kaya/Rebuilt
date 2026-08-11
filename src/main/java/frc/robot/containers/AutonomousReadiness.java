package frc.robot.containers;

/** Pure preflight contract for the mechanisms used by the deployed autonomous routine. */
final class AutonomousReadiness {
  record Inputs(
      boolean pathPlannerConfigured,
      boolean chooserOperational,
      boolean swerveReady,
      boolean turretReady,
      boolean visionReady,
      boolean allianceKnown,
      boolean shooterReady,
      boolean conveyorReady,
      boolean feederReady) {}

  record Result(boolean ready, String reason) {}

  private AutonomousReadiness() {}

  static Result evaluate(Inputs inputs) {
    if (inputs == null) {
      return new Result(false, "readiness inputs unavailable");
    }
    if (!inputs.pathPlannerConfigured()) {
      return new Result(false, "PathPlanner AutoBuilder is not configured");
    }
    if (!inputs.chooserOperational()) {
      return new Result(false, "autonomous chooser failed to load");
    }
    if (!inputs.swerveReady()) {
      return new Result(false, "one or more swerve CAN devices are unavailable");
    }
    if (!inputs.turretReady()) {
      return new Result(false, "turret controller/reference is unavailable");
    }
    if (!inputs.visionReady()) {
      return new Result(false, "turret Limelight heartbeat/pipeline is unavailable");
    }
    if (!inputs.allianceKnown()) {
      return new Result(false, "Driver Station alliance is unknown");
    }
    if (!inputs.shooterReady()) {
      return new Result(false, "shooter controllers/reference are unavailable");
    }
    if (!inputs.conveyorReady()) {
      return new Result(false, "conveyor controller is unavailable");
    }
    if (!inputs.feederReady()) {
      return new Result(false, "feeder is unavailable or blocked by a known fault");
    }
    return new Result(true, "all deployed autonomous dependencies are ready");
  }
}
