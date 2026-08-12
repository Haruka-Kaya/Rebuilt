package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.constants.ConfiguredOperatorActions.Action;
import frc.robot.utils.OperatorActionEvidence.State;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class OperatorActionEvidenceTest {
  @Test
  void publishesSeparatedCanRolesAndStableMultiReasonEvidence() {
    Map<String, String> values = new LinkedHashMap<>();
    AtomicInteger publishCount = new AtomicInteger();
    OperatorActionEvidence evidence = new OperatorActionEvidence((key, value) -> {
      values.put(key, value);
      publishCount.incrementAndGet();
    });

    assertTrue(values.get(OperatorActionEvidence.CONFIGURED_KEY)
        .contains("FIRE{output=[32, 33], dependency=[36, 37, 38]}"));
    assertEquals(
        "[32, 33]", values.get(OperatorActionEvidence.outputCanIdsKey(Action.FIRE)));
    assertEquals(
        "[36, 37, 38]",
        values.get(OperatorActionEvidence.dependencyCanIdsKey(Action.FIRE)));
    assertEquals("NONE STOPPED", values.get(OperatorActionEvidence.CURRENT_STATE_KEY));
    assertTrue(values.get(OperatorActionEvidence.EVIDENCE_KEY)
        .startsWith("DRIVE=STOPPED[NOT_PRESSED]"));
    assertTrue(values.get(OperatorActionEvidence.EVIDENCE_KEY)
        .endsWith("AUTO_AIM=STOPPED[NOT_PRESSED]"));

    var requested = evidence.requested(Action.FIRE);
    var blocked = evidence.blocked(
        Action.FIRE,
        List.of(" feeder known stall ", "shooter not ready", "feeder known stall"));
    int writesAfterBlocked = publishCount.get();
    var duplicate = evidence.blocked(
        Action.FIRE, "feeder known stall", "shooter not ready", "feeder known stall");
    int writesAfterDuplicate = publishCount.get();
    var stopped = evidence.stopped(Action.FIRE, "input released or command ended");

    assertEquals(State.PRESSED, requested.state());
    assertEquals(State.BLOCKED, blocked.state());
    assertEquals(
        List.of("FEEDER_KNOWN_STALL", "SHOOTER_NOT_READY"), blocked.reasons());
    assertEquals("FEEDER_KNOWN_STALL + SHOOTER_NOT_READY", blocked.reason());
    assertSame(blocked, duplicate);
    assertEquals(writesAfterBlocked, writesAfterDuplicate);
    assertEquals(State.STOPPED, stopped.state());
    assertEquals("INPUT_RELEASED_OR_COMMAND_ENDED", stopped.reason());
    assertEquals("FIRE STOPPED", values.get(OperatorActionEvidence.CURRENT_STATE_KEY));
    assertEquals(
        "INPUT_RELEASED_OR_COMMAND_ENDED",
        values.get(OperatorActionEvidence.CURRENT_REASON_KEY));
    assertTrue(values.get(OperatorActionEvidence.EVIDENCE_KEY)
        .contains("FIRE=STOPPED[INPUT_RELEASED_OR_COMMAND_ENDED]"));
  }

  @Test
  @SuppressWarnings("deprecation")
  void compatibilityStatesMapToActiveOrBlockedWithoutClaimingMotion() {
    OperatorActionEvidence evidence = new OperatorActionEvidence((key, value) -> {});

    assertEquals(State.BLOCKED, evidence.faulted(Action.DRIVE, "  ").state());
    assertEquals("UNSPECIFIED", evidence.snapshot(Action.DRIVE).reason());
    assertEquals(State.ACTIVE, evidence.waiting(Action.REV, null).state());
    assertEquals("UNSPECIFIED", evidence.snapshot(Action.REV).reason());
    assertEquals(
        State.ACTIVE,
        evidence.partial(Action.REV, "flywheels commanded hood unreferenced").state());
  }

  @Test
  void publishesOnlyTopicsWhoseValuesChanged() {
    AtomicInteger publishCount = new AtomicInteger();
    OperatorActionEvidence evidence = new OperatorActionEvidence(
        (key, value) -> publishCount.incrementAndGet());

    evidence.blocked(Action.DRIVE, "first blocker");
    int writesBeforeReasonChange = publishCount.get();
    evidence.blocked(Action.DRIVE, "second blocker");
    assertEquals(writesBeforeReasonChange + 3, publishCount.get());

    int writesBeforeDuplicate = publishCount.get();
    var prior = evidence.snapshot(Action.DRIVE);
    var duplicate = evidence.blocked(Action.DRIVE, "second blocker");
    assertSame(prior, duplicate);
    assertEquals(writesBeforeDuplicate, publishCount.get());
  }

  @Test
  void commandEndPreservesReleaseOrBlockAndLifecycleStopClearsAllActions() {
    Map<String, String> values = new LinkedHashMap<>();
    OperatorActionEvidence evidence = new OperatorActionEvidence(values::put);

    evidence.blocked(Action.FIRE, "known stall");
    assertEquals(
        State.BLOCKED,
        evidence.commandEnded(Action.FIRE, true).state(),
        "whileTrue cancellation must not overwrite a precise blocker");

    evidence.stopped(Action.REV, "input released");
    assertEquals(
        "INPUT_RELEASED",
        evidence.commandEnded(Action.REV, true).reason(),
        "whileTrue reports a normal release as interrupted");

    evidence.active(Action.DRIVE, "request accepted");
    evidence.allStopped("robot output stop requested");
    for (Action action : Action.values()) {
      assertEquals(State.STOPPED, evidence.snapshot(action).state());
      assertEquals("ROBOT_OUTPUT_STOP_REQUESTED", evidence.snapshot(action).reason());
    }
    assertEquals("ALL STOPPED", values.get(OperatorActionEvidence.CURRENT_STATE_KEY));
  }
}
