package frc.robot.diagnostics;

/** Latches the first stop-barrier failure so no later Hardware Self-Test stage can move. */
public final class SelfTestRunState {
  private boolean aborted;
  private String abortReason = "";

  public boolean mayContinue() {
    return !aborted;
  }

  public void abort(String reason) {
    if (!aborted) {
      aborted = true;
      abortReason = reason == null || reason.isBlank() ? "UNSPECIFIED" : reason;
    }
  }

  public String abortReason() {
    return abortReason;
  }
}
