package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SelfTestRunStateTest {
  @Test
  void firstStopFailurePermanentlyBlocksLaterMotionStages() {
    SelfTestRunState state = new SelfTestRunState();

    assertTrue(state.mayContinue());
    state.abort("ID31 stop timeout");
    state.abort("later failure");

    assertFalse(state.mayContinue());
    assertEquals("ID31 stop timeout", state.abortReason());
  }

  @Test
  void blankReasonIsStillExplicit() {
    SelfTestRunState state = new SelfTestRunState();

    state.abort(" ");

    assertEquals("UNSPECIFIED", state.abortReason());
  }
}
