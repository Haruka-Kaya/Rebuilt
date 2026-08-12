package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.constants.ConfiguredOperatorActions;
import frc.robot.constants.ConfiguredOperatorActions.Action;
import frc.robot.constants.ConfiguredOperatorActions.ActionSpec;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class ConfiguredOperatorActionsTest {
  @Test
  void manifestCoversEveryProductionOperatorActionExactlyOnce() {
    assertEquals(List.of(Action.values()), ConfiguredOperatorActions.actions());
    assertEquals(10, ConfiguredOperatorActions.specs().size());
    assertEquals(
        ConfiguredOperatorActions.actions(),
        ConfiguredOperatorActions.specs().stream().map(ActionSpec::action).toList());

    assertEquals(
        List.of(30, 31, 33),
        ConfiguredOperatorActions.spec(Action.INTAKE).outputCanIds());
    assertEquals(
        List.of(32, 33),
        ConfiguredOperatorActions.spec(Action.FIRE).outputCanIds());
    assertEquals(
        List.of(36, 37, 38),
        ConfiguredOperatorActions.spec(Action.FIRE).dependencyCanIds());
    assertEquals(
        List.of(50, 51, 52, 53, 54, 55, 56, 57),
        ConfiguredOperatorActions.spec(Action.DRIVE).outputCanIds());
    assertEquals(
        List.of(20, 40, 41, 42, 43),
        ConfiguredOperatorActions.spec(Action.DRIVE).dependencyCanIds());
    assertEquals(List.of(), ConfiguredOperatorActions.spec(Action.SEED_FIELD).outputCanIds());
    assertEquals(
        List.of(20), ConfiguredOperatorActions.spec(Action.SEED_FIELD).dependencyCanIds());
    assertTrue(ConfiguredOperatorActions.configuredSummary()
        .contains("AUTO_AIM{output=[39], dependency=[]}"));
  }

  @Test
  void actionTopicsAndCanRoleListsRemainStable() {
    assertEquals("JUMP BUMP", Action.JUMP_BUMP.topicKey());
    assertEquals("AUTO AIM", Action.AUTO_AIM.topicKey());
    for (ActionSpec actionSpec : ConfiguredOperatorActions.specs()) {
      assertTrue(!actionSpec.label().isBlank());
      assertEquals(
          actionSpec.outputCanIds().stream().distinct().sorted().toList(),
          actionSpec.outputCanIds());
      assertEquals(
          actionSpec.dependencyCanIds().stream().distinct().sorted().toList(),
          actionSpec.dependencyCanIds());
      assertTrue(actionSpec.outputCanIds().stream()
          .noneMatch(actionSpec.dependencyCanIds()::contains));
    }
  }

  @Test
  void everyActionKeepsItsExactOutputAndDependencyCanRoles() {
    Map<Action, List<Integer>> outputs = Map.ofEntries(
        Map.entry(Action.DRIVE, List.of(50, 51, 52, 53, 54, 55, 56, 57)),
        Map.entry(Action.WHEEL_LOCK, List.of(50, 51, 52, 53, 54, 55, 56, 57)),
        Map.entry(Action.SEED_FIELD, List.of()),
        Map.entry(Action.JUMP_BUMP, List.of(50, 51, 52, 53, 54, 55, 56, 57)),
        Map.entry(Action.INTAKE, List.of(30, 31, 33)),
        Map.entry(Action.OUTPUT, List.of(30, 31, 33)),
        Map.entry(Action.RETRACT, List.of(30)),
        Map.entry(Action.REV, List.of(36, 37, 38)),
        Map.entry(Action.FIRE, List.of(32, 33)),
        Map.entry(Action.AUTO_AIM, List.of(39)));
    Map<Action, List<Integer>> dependencies = Map.ofEntries(
        Map.entry(Action.DRIVE, List.of(20, 40, 41, 42, 43)),
        Map.entry(Action.WHEEL_LOCK, List.of(40, 41, 42, 43)),
        Map.entry(Action.SEED_FIELD, List.of(20)),
        Map.entry(Action.JUMP_BUMP, List.of(20, 40, 41, 42, 43)),
        Map.entry(Action.INTAKE, List.of()),
        Map.entry(Action.OUTPUT, List.of()),
        Map.entry(Action.RETRACT, List.of()),
        Map.entry(Action.REV, List.of()),
        Map.entry(Action.FIRE, List.of(36, 37, 38)),
        Map.entry(Action.AUTO_AIM, List.of()));

    for (Action action : Action.values()) {
      assertEquals(outputs.get(action), ConfiguredOperatorActions.spec(action).outputCanIds());
      assertEquals(
          dependencies.get(action), ConfiguredOperatorActions.spec(action).dependencyCanIds());
    }
  }
}
