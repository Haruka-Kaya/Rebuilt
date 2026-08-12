package frc.robot.containers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.containers.AutonomousRunState.Phase;
import frc.robot.utils.ProcessOutputSafety;
import frc.robot.utils.RobotOutputSafetySupervisor;
import org.junit.jupiter.api.Test;

class AutonomousOutputSafetyPolicyTest {
  @Test
  void onlyArmedAndProcessAuthorizedMayContinueAnActiveAuto() {
    for (RobotOutputSafetySupervisor.Phase phase
        : RobotOutputSafetySupervisor.Phase.values()) {
      var result = AutoContainer.evaluateActiveOutputSafety(
          snapshot(phase, true, "supervisor-" + phase, "process-authorized"));
      if (phase == RobotOutputSafetySupervisor.Phase.ARMED) {
        assertTrue(result.ready());
      } else {
        assertFalse(result.ready(), () -> phase + " must abort an active auto");
        assertTrue(result.reason().contains(phase.name()));
      }
    }

    var revoked = AutoContainer.evaluateActiveOutputSafety(
        snapshot(
            RobotOutputSafetySupervisor.Phase.ARMED,
            false,
            "supervisor-armed",
            "heartbeat-generation-revoked"));
    assertFalse(revoked.ready());
    assertTrue(revoked.reason().contains("heartbeat-generation-revoked"));

    assertFalse(AutoContainer.evaluateActiveOutputSafety(null).ready());
  }

  @Test
  void outputSafetyAbortCannotBecomeCompletedDuringFinallyClassification() {
    AutonomousRunState state = new AutonomousRunState();
    state.started();
    var revoked = AutoContainer.evaluateActiveOutputSafety(
        snapshot(
            RobotOutputSafetySupervisor.Phase.TRIPPED,
            false,
            "heartbeat-expired-stop-confirmed",
            "heartbeat-expired"));

    if (!revoked.ready()) {
      assertTrue(state.abort());
    }

    assertEquals(Phase.ABORTED, state.finished(true));
  }

  private static RobotOutputSafetySupervisor.Snapshot snapshot(
      RobotOutputSafetySupervisor.Phase phase,
      boolean outputAuthorized,
      String supervisorReason,
      String processReason) {
    return new RobotOutputSafetySupervisor.Snapshot(
        phase,
        supervisorReason,
        0.0,
        1L,
        false,
        "stop-summary",
        null,
        false,
        new ProcessOutputSafety.Snapshot(1L, outputAuthorized, processReason));
  }
}
