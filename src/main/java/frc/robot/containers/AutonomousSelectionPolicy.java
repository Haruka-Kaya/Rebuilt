package frc.robot.containers;

/** Pure preflight policy that requires both healthy dependencies and an explicit auto selection. */
final class AutonomousSelectionPolicy {
  record Result(boolean ready, String reason) {}

  private AutonomousSelectionPolicy() {}

  static Result evaluate(AutonomousReadiness.Result dependencies, String selectedAutoName) {
    if (dependencies == null) {
      return new Result(false, "readiness inputs unavailable");
    }
    if (!dependencies.ready()) {
      return new Result(false, dependencies.reason());
    }
    if (selectedAutoName == null || selectedAutoName.isBlank()) {
      return new Result(false, "select a reviewed autonomous routine");
    }
    return new Result(
        true,
        selectedAutoName.trim() + " selected; " + dependencies.reason());
  }
}
