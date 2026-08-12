package frc.robot.constants;

import java.util.List;

/** Shared PS5 raw-button mapping used by bindings, simulation checks, and operator diagnostics. */
public final class ConfiguredOperatorControls {
  public static final int SQUARE = 1;
  public static final int CROSS = 2;
  public static final int CIRCLE = 3;
  public static final int TRIANGLE = 4;
  public static final int L1 = 5;
  public static final int R1 = 6;
  public static final int L2 = 7;
  public static final int R2 = 8;
  public static final int CREATE = 9;
  public static final int OPTIONS = 10;
  public static final int R3 = 12;
  public static final int TOUCHPAD = 14;

  public static final int DRIVER_INTAKE = L1;
  public static final int DRIVER_OUTPUT = R1;
  public static final int DRIVER_REV = L2;
  public static final int DRIVER_FIRE = R2;
  public static final int DRIVER_SEED_FIELD = CREATE;
  public static final int DRIVER_JUMP_BUMP = R3;
  public static final int DRIVER_WHEEL_LOCK = TOUCHPAD;
  public static final int DRIVER_RETRACT_FALLBACK = SQUARE;
  public static final int DRIVER_AUTO_AIM_FALLBACK = TRIANGLE;

  public static final int OPERATOR_RETRACT = L1;
  public static final int MAINTENANCE_AUTO_AIM = L1;
  public static final int UNHOMED_DIAGNOSTIC_NEGATIVE = L1;
  public static final int UNHOMED_DIAGNOSTIC_POSITIVE = R1;
  public static final int UNHOMED_DIAGNOSTIC_DEADMAN = CREATE;
  public static final int CLIMBER_DEADMAN = OPTIONS;
  public static final int CLIMBER_LEFT_POSITIVE = SQUARE;
  public static final int CLIMBER_LEFT_NEGATIVE = CROSS;
  public static final int CLIMBER_RIGHT_POSITIVE = CIRCLE;
  public static final int CLIMBER_RIGHT_NEGATIVE = TRIANGLE;

  private static final List<Integer> DRIVER_SAFETY_BUTTONS = List.of(
      DRIVER_RETRACT_FALLBACK,
      DRIVER_AUTO_AIM_FALLBACK,
      DRIVER_INTAKE,
      DRIVER_OUTPUT,
      DRIVER_REV,
      DRIVER_FIRE,
      DRIVER_SEED_FIELD,
      DRIVER_JUMP_BUMP,
      DRIVER_WHEEL_LOCK);

  private static final List<Integer> CLIMBER_FACE_BUTTONS = List.of(
      CLIMBER_LEFT_POSITIVE,
      CLIMBER_LEFT_NEGATIVE,
      CLIMBER_RIGHT_POSITIVE,
      CLIMBER_RIGHT_NEGATIVE);

  private ConfiguredOperatorControls() {}

  public static List<Integer> driverSafetyButtons() {
    return DRIVER_SAFETY_BUTTONS;
  }

  public static List<Integer> climberFaceButtons() {
    return CLIMBER_FACE_BUTTONS;
  }

  public static int maximumDriverButton() {
    return DRIVER_SAFETY_BUTTONS.stream().mapToInt(Integer::intValue).max().orElseThrow();
  }

  public static int maximumOperatorButton() {
    return OPERATOR_RETRACT;
  }

  public static int maximumMaintenanceButton() {
    return Math.max(MAINTENANCE_AUTO_AIM, CLIMBER_DEADMAN);
  }

  public static String configuredSummary() {
    return "Driver L1=intake R1=output L2=rev R2=fire Create=seed R3=jump-bump "
        + "Touchpad=wheel-lock Square=retract-fallback Triangle=auto-aim-fallback; "
        + "Operator L1=retract; Maintenance L1=auto-aim; "
        + "Test unhomed actuator=Maintenance Create+(L1-/R1+); "
        + "Test climber=Options+(Square/Cross/Circle/Triangle)";
  }
}
