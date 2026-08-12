package frc.robot.constants;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/** Static operator-action manifest; live health and interlock results are published separately. */
public final class ConfiguredOperatorActions {
  public enum Action {
    DRIVE,
    WHEEL_LOCK,
    SEED_FIELD,
    JUMP_BUMP,
    INTAKE,
    OUTPUT,
    RETRACT,
    REV,
    FIRE,
    AUTO_AIM;

    public String topicKey() {
      return name().replace('_', ' ');
    }
  }

  /**
   * CAN devices involved in an operator action.
   *
   * @param outputCanIds devices which the action may command
   * @param dependencyCanIds devices whose data/readiness may gate the action but which it does not
   *     command
   */
  public record ActionSpec(
      Action action,
      String label,
      List<Integer> outputCanIds,
      List<Integer> dependencyCanIds) {
    public ActionSpec {
      Objects.requireNonNull(action, "action");
      Objects.requireNonNull(label, "label");
      if (label.isBlank()) {
        throw new IllegalArgumentException("label must not be blank");
      }
      outputCanIds = normalizedIds(outputCanIds);
      dependencyCanIds = normalizedIds(dependencyCanIds);
      if (outputCanIds.stream().anyMatch(dependencyCanIds::contains)) {
        throw new IllegalArgumentException(
            action + " cannot list a CAN ID as both output and dependency");
      }
    }
  }

  private static final List<ActionSpec> ACTION_SPECS = List.of(
      spec(
          Action.DRIVE,
          "Drive",
          ids(50, 57),
          concat(
              List.of(ConfiguredCanHardware.PIGEON_ID),
              ConfiguredCanHardware.swerveEncoderIds())),
      spec(
          Action.WHEEL_LOCK,
          "Wheel Lock",
          ids(50, 57),
          ConfiguredCanHardware.swerveEncoderIds()),
      spec(
          Action.SEED_FIELD,
          "Seed Field",
          List.of(),
          List.of(ConfiguredCanHardware.PIGEON_ID)),
      spec(
          Action.JUMP_BUMP,
          "Jump Bump",
          ids(50, 57),
          concat(
              List.of(ConfiguredCanHardware.PIGEON_ID),
              ConfiguredCanHardware.swerveEncoderIds())),
      spec(
          Action.INTAKE,
          "Intake",
          List.of(
              ConfiguredCanHardware.INTAKE_ACTUATOR_ID,
              ConfiguredCanHardware.INTAKE_ROLLER_ID,
              ConfiguredCanHardware.CONVEYOR_ID),
          List.of()),
      spec(
          Action.OUTPUT,
          "Output",
          List.of(
              ConfiguredCanHardware.INTAKE_ACTUATOR_ID,
              ConfiguredCanHardware.INTAKE_ROLLER_ID,
              ConfiguredCanHardware.CONVEYOR_ID),
          List.of()),
      spec(
          Action.RETRACT,
          "Retract",
          List.of(ConfiguredCanHardware.INTAKE_ACTUATOR_ID),
          List.of()),
      spec(
          Action.REV,
          "Rev",
          List.of(
              ConfiguredCanHardware.SHOOTER_LEADER_ID,
              ConfiguredCanHardware.SHOOTER_FOLLOWER_ID,
              ConfiguredCanHardware.SHOOTER_ACTUATOR_ID),
          List.of()),
      spec(
          Action.FIRE,
          "Fire",
          List.of(ConfiguredCanHardware.FEEDER_ID, ConfiguredCanHardware.CONVEYOR_ID),
          List.of(
              ConfiguredCanHardware.SHOOTER_LEADER_ID,
              ConfiguredCanHardware.SHOOTER_FOLLOWER_ID,
              ConfiguredCanHardware.SHOOTER_ACTUATOR_ID)),
      spec(
          Action.AUTO_AIM,
          "Auto Aim",
          List.of(ConfiguredCanHardware.TURRET_ID),
          List.of()));

  private static final Map<Action, ActionSpec> SPECS_BY_ACTION = ACTION_SPECS.stream()
      .collect(Collectors.toUnmodifiableMap(ActionSpec::action, Function.identity()));

  static {
    if (SPECS_BY_ACTION.size() != Action.values().length) {
      throw new IllegalStateException("Every operator action must have exactly one ActionSpec");
    }
  }

  private ConfiguredOperatorActions() {}

  public static List<ActionSpec> specs() {
    return ACTION_SPECS;
  }

  public static ActionSpec spec(Action action) {
    ActionSpec result = SPECS_BY_ACTION.get(Objects.requireNonNull(action, "action"));
    if (result == null) {
      throw new IllegalArgumentException("No ActionSpec for " + action);
    }
    return result;
  }

  public static List<Action> actions() {
    return ACTION_SPECS.stream().map(ActionSpec::action).toList();
  }

  public static String configuredSummary() {
    return ACTION_SPECS.stream()
        .map(actionSpec -> actionSpec.action().name()
            + "{output=" + actionSpec.outputCanIds()
            + ", dependency=" + actionSpec.dependencyCanIds() + "}")
        .collect(Collectors.joining("; "));
  }

  private static ActionSpec spec(
      Action action,
      String label,
      List<Integer> outputCanIds,
      List<Integer> dependencyCanIds) {
    return new ActionSpec(action, label, outputCanIds, dependencyCanIds);
  }

  private static List<Integer> normalizedIds(List<Integer> canIds) {
    Objects.requireNonNull(canIds, "canIds");
    if (canIds.stream().anyMatch(Objects::isNull)) {
      throw new IllegalArgumentException("CAN IDs must not contain null");
    }
    return canIds.stream().distinct().sorted().toList();
  }

  private static List<Integer> ids(int firstInclusive, int lastInclusive) {
    return IntStream.rangeClosed(firstInclusive, lastInclusive).boxed().toList();
  }

  @SafeVarargs
  private static List<Integer> concat(List<Integer>... parts) {
    return Arrays.stream(parts).flatMap(List::stream).distinct().sorted().toList();
  }
}
