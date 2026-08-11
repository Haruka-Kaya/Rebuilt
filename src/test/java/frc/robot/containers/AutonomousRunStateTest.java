package frc.robot.containers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.containers.AutonomousRunState.Phase;
import org.junit.jupiter.api.Test;

class AutonomousRunStateTest {
  @Test
  void naturalCompletionCannotBeRelabeledByALateDependencyFault() {
    AutonomousRunState state = new AutonomousRunState();
    state.started();

    assertEquals(Phase.COMPLETED, state.finished(false));
    assertFalse(state.abort());
    assertEquals(Phase.COMPLETED, state.phase());
  }

  @Test
  void interruptionAndRuntimeAbortRemainDistinct() {
    AutonomousRunState interrupted = new AutonomousRunState();
    interrupted.started();
    assertEquals(Phase.INTERRUPTED, interrupted.finished(true));

    AutonomousRunState aborted = new AutonomousRunState();
    aborted.started();
    assertTrue(aborted.abort());
    assertEquals(Phase.ABORTED, aborted.finished(true));
  }

  @Test
  void blockedSafeStopIsNeverConsideredAnActiveManagedAuto() {
    AutonomousRunState state = new AutonomousRunState();

    assertFalse(state.running());
    assertFalse(state.abort());
    assertEquals(Phase.IDLE, state.phase());
  }

  @Test
  void aSecondAutonomousRunStartsFromRunningAgain() {
    AutonomousRunState state = new AutonomousRunState();
    state.started();
    state.finished(false);

    state.started();

    assertTrue(state.running());
    assertEquals(Phase.RUNNING, state.phase());
  }
}
