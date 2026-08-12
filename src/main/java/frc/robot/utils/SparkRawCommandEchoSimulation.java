package frc.robot.utils;

import frc.robot.constants.ConfiguredCanHardware;
import frc.robot.utils.SparkMAXContainer.FollowerSimulationState;
import frc.robot.utils.SparkSimulationHandle.RawCommandSnapshot;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

/**
 * Deterministic command echo for SPARK simulation without a mechanism or motor physics model.
 *
 * <p>Duty-cycle commands are copied only to applied output, velocity commands are copied only to
 * velocity, and position is always held. Consequently this class cannot establish an encoder
 * reference or claim that a mechanism moved. All faults, reset observations, disabled state, and
 * unsupported raw control modes stop the echoed output.
 */
public final class SparkRawCommandEchoSimulation {
  public static final String SOURCE = "RAW_COMMAND_ECHO_NO_PHYSICS_NO_REFERENCE";

  static final int DUTY_CYCLE_CONTROL_MODE = 0;
  static final int VELOCITY_CONTROL_MODE = 1;
  static final int RESET_WARNING_MASK = 1 << 6;
  static final int SHOOTER_LEADER_CAN_ID = ConfiguredCanHardware.SHOOTER_LEADER_ID;
  static final int INVERTED_SHOOTER_FOLLOWER_CAN_ID =
      ConfiguredCanHardware.SHOOTER_FOLLOWER_ID;

  private SparkRawCommandEchoSimulation() {}

  /**
   * Steps every currently registered configured SPARK, with leader 36 before inverted follower 37.
   *
   * <p>Missing handles are intentionally skipped so partial-subsystem simulation and unit tests do
   * not synthesize hardware that the robot process never constructed.
   */
  public static List<Response> stepConfiguredControllers(boolean enabled) {
    List<Response> responses = new ArrayList<>();
    Optional<Response> shooterLeaderResponse = SparkMAXContainer
        .getSimulationHandleForId(SHOOTER_LEADER_CAN_ID)
        .map(handle -> step(handle, enabled));
    shooterLeaderResponse.ifPresent(responses::add);

    for (int canId : ConfiguredCanHardware.sparkDeviceIds()) {
      if (canId == SHOOTER_LEADER_CAN_ID || canId == INVERTED_SHOOTER_FOLLOWER_CAN_ID) {
        continue;
      }
      SparkMAXContainer.getSimulationHandleForId(canId)
          .map(handle -> step(handle, enabled))
          .ifPresent(responses::add);
    }

    SparkMAXContainer.getSimulationHandleForId(INVERTED_SHOOTER_FOLLOWER_CAN_ID)
        .map(handle -> step(
            handle,
            enabled,
            shooterLeaderResponse,
            followerMode(SparkMAXContainer
                .getFollowerSimulationStateForId(INVERTED_SHOOTER_FOLLOWER_CAN_ID))))
        .ifPresent(responses::add);
    return List.copyOf(responses);
  }

  /**
   * Observes and applies one response without allowing a setpoint write to interleave.
   *
   * @param handle simulation-only SPARK façade
   * @param enabled whether robot output is currently enabled
   * @param followerLeaderResponse leader response when stepping configured inverted follower 37
   * @return the applied non-physical response and its explicit provenance
   */
  public static Response step(
      SparkSimulationHandle handle,
      boolean enabled,
      Optional<Response> followerLeaderResponse) {
    return step(handle, enabled, followerLeaderResponse, FollowerEchoMode.UNVERIFIED_STOP);
  }

  static Response step(
      SparkSimulationHandle handle,
      boolean enabled,
      Optional<Response> followerLeaderResponse,
      FollowerEchoMode followerMode) {
    Objects.requireNonNull(handle, "handle");
    Objects.requireNonNull(followerLeaderResponse, "followerLeaderResponse");
    Objects.requireNonNull(followerMode, "followerMode");
    return handle.applyRawCommandEcho(
        command -> evaluate(command, enabled, followerLeaderResponse, followerMode));
  }

  /** Convenience overload for an independently commanded SPARK. */
  public static Response step(SparkSimulationHandle handle, boolean enabled) {
    return step(handle, enabled, Optional.empty());
  }

  /** Pure response calculation; it performs no vendor, HAL, clock, or dashboard access. */
  static Response evaluate(
      RawCommandSnapshot command,
      boolean enabled,
      Optional<Response> followerLeaderResponse) {
    return evaluate(
        command, enabled, followerLeaderResponse, FollowerEchoMode.UNVERIFIED_STOP);
  }

  static Response evaluate(
      RawCommandSnapshot command,
      boolean enabled,
      Optional<Response> followerLeaderResponse,
      FollowerEchoMode followerMode) {
    Objects.requireNonNull(command, "command");
    Objects.requireNonNull(followerLeaderResponse, "followerLeaderResponse");
    Objects.requireNonNull(followerMode, "followerMode");

    double busVoltage = Double.isFinite(command.busVoltage()) ? command.busVoltage() : 0.0;
    if (command.activeFaultBits() != 0 || command.stickyFaultBits() != 0) {
      return stopped(command, busVoltage, Status.FAULT_STOP);
    }
    if (((command.activeWarningBits() | command.stickyWarningBits()) & RESET_WARNING_MASK) != 0) {
      return stopped(command, busVoltage, Status.RESET_STOP);
    }
    if (!enabled) {
      return stopped(command, busVoltage, Status.DISABLED_STOP);
    }
    if (!Double.isFinite(command.setpoint())) {
      return stopped(command, busVoltage, Status.UNKNOWN_CONTROL_STOP);
    }

    if (command.canId() == INVERTED_SHOOTER_FOLLOWER_CAN_ID) {
      if (followerMode == FollowerEchoMode.ISOLATED_DIRECT) {
        return independentlyCommanded(command, busVoltage);
      }
      if (followerMode != FollowerEchoMode.NORMAL_INVERTED) {
        return stopped(command, busVoltage, Status.FOLLOWER_STATE_UNVERIFIED_STOP);
      }
      if (followerLeaderResponse.isEmpty()) {
        return stopped(command, busVoltage, Status.FOLLOWER_LEADER_UNAVAILABLE_STOP);
      }
      Response leader = followerLeaderResponse.orElseThrow();
      if (leader.canId() != SHOOTER_LEADER_CAN_ID
          || !SOURCE.equals(leader.source())
          || !allFinite(
              leader.appliedOutput(),
              leader.motorCurrentAmps(),
              leader.velocity(),
              leader.position(),
              leader.busVoltage())) {
        return stopped(command, busVoltage, Status.UNKNOWN_CONTROL_STOP);
      }
      return new Response(
          command.canId(),
          negateCanonical(leader.appliedOutput()),
          0.0,
          negateCanonical(leader.velocity()),
          command.position(),
          busVoltage,
          Status.INVERTED_FOLLOWER_ECHO,
          SOURCE);
    }

    return independentlyCommanded(command, busVoltage);
  }

  private static Response independentlyCommanded(
      RawCommandSnapshot command, double busVoltage) {
    return switch (command.rawControlMode()) {
      case DUTY_CYCLE_CONTROL_MODE -> Math.abs(command.setpoint()) <= 1e-9
          ? stopped(command, busVoltage, Status.ZERO_COMMAND)
          : new Response(
              command.canId(),
              clampDutyCycle(command.setpoint()),
              0.0,
              0.0,
              command.position(),
              busVoltage,
              Status.DUTY_CYCLE_ECHO,
              SOURCE);
      case VELOCITY_CONTROL_MODE -> Math.abs(command.setpoint()) <= 1e-9
          ? stopped(command, busVoltage, Status.ZERO_COMMAND)
          : new Response(
              command.canId(),
              0.0,
              0.0,
              command.setpoint(),
              command.position(),
              busVoltage,
              Status.VELOCITY_ECHO,
              SOURCE);
      default -> stopped(command, busVoltage, Status.UNKNOWN_CONTROL_STOP);
    };
  }

  static FollowerEchoMode followerMode(Optional<FollowerSimulationState> state) {
    if (state.isEmpty() || !state.orElseThrow().baseReady()) {
      return FollowerEchoMode.UNVERIFIED_STOP;
    }
    FollowerSimulationState follower = state.orElseThrow();
    if (follower.expectedFollower()
        && follower.isolatedDiagnosticActive()
        && !follower.cachedFollower()) {
      return FollowerEchoMode.ISOLATED_DIRECT;
    }
    if (follower.expectedFollower()
        && follower.transitionIdle()
        && follower.cachedFollower()) {
      return FollowerEchoMode.NORMAL_INVERTED;
    }
    return FollowerEchoMode.UNVERIFIED_STOP;
  }

  private static Response stopped(
      RawCommandSnapshot command, double busVoltage, Status status) {
    return new Response(
        command.canId(), 0.0, 0.0, 0.0, command.position(), busVoltage, status, SOURCE);
  }

  private static double clampDutyCycle(double dutyCycle) {
    return Math.max(-1.0, Math.min(1.0, dutyCycle));
  }

  private static double negateCanonical(double value) {
    return value == 0.0 ? 0.0 : -value;
  }

  private static boolean allFinite(double... values) {
    for (double value : values) {
      if (!Double.isFinite(value)) {
        return false;
      }
    }
    return true;
  }

  public enum Status {
    DUTY_CYCLE_ECHO,
    VELOCITY_ECHO,
    ZERO_COMMAND,
    INVERTED_FOLLOWER_ECHO,
    DISABLED_STOP,
    FAULT_STOP,
    RESET_STOP,
    UNKNOWN_CONTROL_STOP,
    FOLLOWER_LEADER_UNAVAILABLE_STOP,
    FOLLOWER_STATE_UNVERIFIED_STOP
  }

  enum FollowerEchoMode {
    NORMAL_INVERTED,
    ISOLATED_DIRECT,
    UNVERIFIED_STOP
  }

  /** Immutable response applied directly to raw REV simulation telemetry. */
  public record Response(
      int canId,
      double appliedOutput,
      double motorCurrentAmps,
      double velocity,
      double position,
      double busVoltage,
      Status status,
      String source) {}
}
