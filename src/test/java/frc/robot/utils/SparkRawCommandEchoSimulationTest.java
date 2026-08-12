package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.utils.SparkRawCommandEchoSimulation.Response;
import frc.robot.utils.SparkRawCommandEchoSimulation.Status;
import frc.robot.utils.SparkMAXContainer.FollowerSimulationState;
import frc.robot.utils.SparkSimulationHandle.RawCommandSnapshot;
import frc.robot.utils.SparkSimulationHandle.RawTelemetry;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class SparkRawCommandEchoSimulationTest {
  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void officialRawModesEchoOnlyDimensionallyMatchingTelemetryAndHoldPosition() {
    assertAll(
        () -> assertEquals(0, ControlType.kDutyCycle.value),
        () -> assertEquals(1, ControlType.kVelocity.value),
        () -> assertEquals(
            ControlType.kDutyCycle.value,
            SparkRawCommandEchoSimulation.DUTY_CYCLE_CONTROL_MODE),
        () -> assertEquals(
            ControlType.kVelocity.value,
            SparkRawCommandEchoSimulation.VELOCITY_CONTROL_MODE),
        () -> assertTrue(new Faults(1 << 3).can),
        () -> assertTrue(
            new Warnings(SparkRawCommandEchoSimulation.RESET_WARNING_MASK).hasReset));

    Response duty = evaluate(command(31, ControlType.kDutyCycle.value, 0.35), true);
    Response clampedDuty = evaluate(command(31, ControlType.kDutyCycle.value, -1.5), true);
    Response velocity = evaluate(command(31, ControlType.kVelocity.value, -2750.0), true);

    assertAll(
        () -> assertEquals(Status.DUTY_CYCLE_ECHO, duty.status()),
        () -> assertEquals(0.35, duty.appliedOutput()),
        () -> assertEquals(0.0, duty.velocity()),
        () -> assertEquals(17.25, duty.position()),
        () -> assertEquals(-1.0, clampedDuty.appliedOutput()),
        () -> assertEquals(Status.VELOCITY_ECHO, velocity.status()),
        () -> assertEquals(0.0, velocity.appliedOutput()),
        () -> assertEquals(-2750.0, velocity.velocity()),
        () -> assertEquals(17.25, velocity.position()),
        () -> assertEquals(0.0, velocity.motorCurrentAmps()),
        () -> assertEquals(
            "RAW_COMMAND_ECHO_NO_PHYSICS_NO_REFERENCE", velocity.source()));
  }

  @Test
  void zeroFaultResetDisabledAndUnknownModesAlwaysStopAndHoldPosition() {
    RawCommandSnapshot base = command(32, ControlType.kDutyCycle.value, 0.4);

    assertStopped(evaluate(command(32, ControlType.kDutyCycle.value, 0.0), true),
        Status.ZERO_COMMAND);
    assertStopped(evaluate(withFaults(base, 1, 0), true), Status.FAULT_STOP);
    assertStopped(evaluate(withFaults(base, 0, 1 << 3), true), Status.FAULT_STOP);
    assertStopped(evaluate(withWarnings(base, 1 << 6, 0), true), Status.RESET_STOP);
    assertStopped(evaluate(withWarnings(base, 0, 1 << 6), true), Status.RESET_STOP);
    assertStopped(evaluate(base, false), Status.DISABLED_STOP);
    assertStopped(evaluate(command(32, ControlType.kPosition.value, 123.0), true),
        Status.UNKNOWN_CONTROL_STOP);
    assertStopped(evaluate(command(32, 99, 0.4), true), Status.UNKNOWN_CONTROL_STOP);
    assertStopped(evaluate(command(32, ControlType.kDutyCycle.value, Double.NaN), true),
        Status.UNKNOWN_CONTROL_STOP);
  }

  @Test
  void id37EchoesOnlyValidatedId36ResponseWithInversionAndItsOwnHeldPosition() {
    Response leaderDuty = evaluate(command(36, ControlType.kDutyCycle.value, 0.08), true);
    Response followerDuty = SparkRawCommandEchoSimulation.evaluate(
        command(37, 99, 999.0),
        true,
        Optional.of(leaderDuty),
        SparkRawCommandEchoSimulation.FollowerEchoMode.NORMAL_INVERTED);
    Response leaderVelocity = evaluate(command(36, ControlType.kVelocity.value, 3200.0), true);
    Response followerVelocity = SparkRawCommandEchoSimulation.evaluate(
        command(37, 99, 999.0),
        true,
        Optional.of(leaderVelocity),
        SparkRawCommandEchoSimulation.FollowerEchoMode.NORMAL_INVERTED);
    Response isolatedDuty = SparkRawCommandEchoSimulation.evaluate(
        command(37, ControlType.kDutyCycle.value, 0.08),
        true,
        Optional.of(leaderDuty),
        SparkRawCommandEchoSimulation.FollowerEchoMode.ISOLATED_DIRECT);

    assertAll(
        () -> assertEquals(Status.INVERTED_FOLLOWER_ECHO, followerDuty.status()),
        () -> assertEquals(-0.08, followerDuty.appliedOutput()),
        () -> assertEquals(0.0, followerDuty.velocity()),
        () -> assertEquals(17.25, followerDuty.position()),
        () -> assertEquals(Status.INVERTED_FOLLOWER_ECHO, followerVelocity.status()),
        () -> assertEquals(0.0, followerVelocity.appliedOutput()),
        () -> assertEquals(-3200.0, followerVelocity.velocity()),
        () -> assertEquals(17.25, followerVelocity.position()),
        () -> assertEquals(Status.DUTY_CYCLE_ECHO, isolatedDuty.status()),
        () -> assertEquals(0.08, isolatedDuty.appliedOutput()),
        () -> assertEquals(0.0, isolatedDuty.velocity()));

    Response wrongLeader = new Response(
        35, 0.3, 0.0, 0.0, 1.0, 12.0, Status.DUTY_CYCLE_ECHO,
        SparkRawCommandEchoSimulation.SOURCE);
    assertStopped(
        SparkRawCommandEchoSimulation.evaluate(
            command(37, 99, 999.0),
            true,
            Optional.of(wrongLeader),
            SparkRawCommandEchoSimulation.FollowerEchoMode.NORMAL_INVERTED),
        Status.UNKNOWN_CONTROL_STOP);
    assertEquals(
        Status.UNKNOWN_CONTROL_STOP,
        SparkRawCommandEchoSimulation.evaluate(
            command(38, 99, 999.0), true, Optional.of(leaderDuty)).status(),
        "a leader response must not turn arbitrary CAN IDs into followers");
    assertStopped(
        SparkRawCommandEchoSimulation.evaluate(
            command(37, ControlType.kDutyCycle.value, 0.5),
            true,
            Optional.empty(),
            SparkRawCommandEchoSimulation.FollowerEchoMode.NORMAL_INVERTED),
        Status.FOLLOWER_LEADER_UNAVAILABLE_STOP);
    assertStopped(
        SparkRawCommandEchoSimulation.evaluate(
            command(37, ControlType.kDutyCycle.value, 0.5), true, Optional.of(leaderDuty)),
        Status.FOLLOWER_STATE_UNVERIFIED_STOP);
  }

  @Test
  void configuredFollowerStateSelectsNormalMirrorOrIsolatedDirectEchoFailClosed() {
    assertAll(
        () -> assertEquals(
            SparkRawCommandEchoSimulation.FollowerEchoMode.NORMAL_INVERTED,
            SparkRawCommandEchoSimulation.followerMode(Optional.of(
                new FollowerSimulationState(true, true, true, false, true)))),
        () -> assertEquals(
            SparkRawCommandEchoSimulation.FollowerEchoMode.ISOLATED_DIRECT,
            SparkRawCommandEchoSimulation.followerMode(Optional.of(
                new FollowerSimulationState(true, false, false, true, true)))),
        () -> assertEquals(
            SparkRawCommandEchoSimulation.FollowerEchoMode.UNVERIFIED_STOP,
            SparkRawCommandEchoSimulation.followerMode(Optional.of(
                new FollowerSimulationState(true, true, false, false, true)))),
        () -> assertEquals(
            SparkRawCommandEchoSimulation.FollowerEchoMode.UNVERIFIED_STOP,
            SparkRawCommandEchoSimulation.followerMode(Optional.of(
                new FollowerSimulationState(true, false, false, true, false)))),
        () -> assertEquals(
            SparkRawCommandEchoSimulation.FollowerEchoMode.UNVERIFIED_STOP,
            SparkRawCommandEchoSimulation.followerMode(Optional.empty())));
  }

  @Test
  void stepReadsOfficialRawCommandAndAppliesResponseThroughTheHandle() {
    try (SparkMax motor = new SparkMax(62, MotorType.kBrushless)) {
      SparkSimulationHandle handle = new SparkSimulationHandle(motor, new Object());
      handle.injectRawTelemetry(new RawTelemetry(0.0, 2.0, 7.0, 42.5, 11.7));

      assertEquals(
          REVLibError.kOk,
          motor.getClosedLoopController().setSetpoint(0.2, ControlType.kDutyCycle));
      Response duty = SparkRawCommandEchoSimulation.step(handle, true);
      var dutySnapshot = handle.observe();
      assertAll(
          () -> assertEquals(Status.DUTY_CYCLE_ECHO, duty.status()),
          () -> assertEquals(ControlType.kDutyCycle.value, dutySnapshot.rawControlMode()),
          () -> assertEquals(0.2, dutySnapshot.setpoint(), 1e-7),
          () -> assertEquals(0.2, dutySnapshot.appliedOutput(), 1e-7),
          () -> assertEquals(0.0, dutySnapshot.velocity()),
          () -> assertEquals(42.5, dutySnapshot.position()));

      assertEquals(
          REVLibError.kOk,
          motor.getClosedLoopController().setSetpoint(-1800.0, ControlType.kVelocity));
      Response velocity = SparkRawCommandEchoSimulation.step(handle, true);
      var velocitySnapshot = handle.observe();
      assertAll(
          () -> assertEquals(Status.VELOCITY_ECHO, velocity.status()),
          () -> assertEquals(ControlType.kVelocity.value, velocitySnapshot.rawControlMode()),
          () -> assertEquals(-1800.0, velocitySnapshot.setpoint()),
          () -> assertEquals(0.0, velocitySnapshot.appliedOutput()),
          () -> assertEquals(-1800.0, velocitySnapshot.velocity()),
          () -> assertEquals(42.5, velocitySnapshot.position()));

      handle.setCanFault(true, false);
      Response faulted = SparkRawCommandEchoSimulation.step(handle, true);
      var faultedSnapshot = handle.observe();
      assertAll(
          () -> assertEquals(Status.FAULT_STOP, faulted.status()),
          () -> assertTrue((faultedSnapshot.activeFaultBits() & (1 << 3)) != 0),
          () -> assertEquals(0.0, faultedSnapshot.appliedOutput()),
          () -> assertEquals(0.0, faultedSnapshot.velocity()),
          () -> assertEquals(42.5, faultedSnapshot.position()));
      handle.setCanFault(false, false);
    }
  }

  @Test
  void seamObservesRawCommandAndAppliesPureResponseWhileHoldingProvidedIoLock() {
    Object ioLock = new Object();
    try (SparkMax motor = new SparkMax(63, MotorType.kBrushless)) {
      SparkSimulationHandle handle = new SparkSimulationHandle(motor, ioLock);
      handle.injectRawTelemetry(new RawTelemetry(0.4, 3.0, 100.0, 8.25, 12.0));
      assertEquals(
          REVLibError.kOk,
          motor.getClosedLoopController().setSetpoint(0.15, ControlType.kDutyCycle));
      handle.setResetWarning(true, true);

      AtomicBoolean heldDuringEvaluation = new AtomicBoolean();
      AtomicReference<RawCommandSnapshot> observed = new AtomicReference<>();
      Response response = handle.applyRawCommandEcho(command -> {
        heldDuringEvaluation.set(Thread.holdsLock(ioLock));
        observed.set(command);
        return SparkRawCommandEchoSimulation.evaluate(command, true, Optional.empty());
      });

      assertAll(
          () -> assertTrue(heldDuringEvaluation.get()),
          () -> assertEquals(ControlType.kDutyCycle.value, observed.get().rawControlMode()),
          () -> assertTrue((observed.get().activeWarningBits() & (1 << 6)) != 0),
          () -> assertTrue((observed.get().stickyWarningBits() & (1 << 6)) != 0),
          () -> assertEquals(Status.RESET_STOP, response.status()),
          () -> assertEquals(0.0, handle.observe().appliedOutput()),
          () -> assertEquals(0.0, handle.observe().velocity()),
          () -> assertEquals(8.25, handle.observe().position()));
      handle.setResetWarning(false, false);
    }
  }

  @Test
  void configuredCoordinatorSkipsMissingHandlesAndStopsAnUnverifiedFollower() {
    new SparkMAXContainer(37);
    new SparkMAXContainer(31);
    new SparkMAXContainer(36);
    try {
      SparkSimulationHandle follower =
          SparkMAXContainer.getSimulationHandleForId(37).orElseThrow();
      SparkSimulationHandle independent =
          SparkMAXContainer.getSimulationHandleForId(31).orElseThrow();
      SparkSimulationHandle leader =
          SparkMAXContainer.getSimulationHandleForId(36).orElseThrow();
      follower.injectRawTelemetry(new RawTelemetry(0.0, 0.0, 0.0, 37.5, 12.0));
      independent.injectRawTelemetry(new RawTelemetry(0.0, 0.0, 0.0, 31.5, 12.0));
      leader.injectRawTelemetry(new RawTelemetry(0.0, 0.0, 0.0, 36.5, 12.0));
      setRawCommand(36, ControlType.kDutyCycle.value, 0.12);
      setRawCommand(31, ControlType.kVelocity.value, 900.0);
      setRawCommand(37, 99, 999.0);

      List<Response> responses =
          SparkRawCommandEchoSimulation.stepConfiguredControllers(true);

      assertAll(
          () -> assertEquals(List.of(36, 31, 37),
              responses.stream().map(Response::canId).toList()),
          () -> assertEquals(Status.DUTY_CYCLE_ECHO, responses.get(0).status()),
          () -> assertEquals(Status.VELOCITY_ECHO, responses.get(1).status()),
          () -> assertEquals(
              Status.FOLLOWER_STATE_UNVERIFIED_STOP, responses.get(2).status()),
          () -> assertEquals(0.0, responses.get(2).appliedOutput(), 1e-7),
          () -> assertEquals(37.5, responses.get(2).position()),
          () -> assertThrows(UnsupportedOperationException.class,
              () -> responses.add(responses.get(0))));
    } finally {
      assertTrue(SparkMAXContainer.cleanupSimulationDevicesForTesting());
    }
  }

  private static Response evaluate(RawCommandSnapshot command, boolean enabled) {
    return SparkRawCommandEchoSimulation.evaluate(command, enabled, Optional.empty());
  }

  private static void setRawCommand(int canId, int controlMode, double setpoint) {
    SimDeviceSim device = new SimDeviceSim("SPARK MAX [" + canId + "]");
    SimInt mode = device.getInt("Control Mode");
    SimDouble target = device.getDouble("Setpoint");
    assertTrue(mode != null && target != null);
    mode.set(controlMode);
    target.set(setpoint);
  }

  private static RawCommandSnapshot command(int canId, int rawControlMode, double setpoint) {
    return new RawCommandSnapshot(
        canId,
        rawControlMode,
        setpoint,
        0.7,
        9.0,
        800.0,
        17.25,
        12.2,
        0,
        0,
        0,
        0);
  }

  private static RawCommandSnapshot withFaults(
      RawCommandSnapshot command, int activeFaultBits, int stickyFaultBits) {
    return new RawCommandSnapshot(
        command.canId(),
        command.rawControlMode(),
        command.setpoint(),
        command.appliedOutput(),
        command.motorCurrentAmps(),
        command.velocity(),
        command.position(),
        command.busVoltage(),
        activeFaultBits,
        stickyFaultBits,
        command.activeWarningBits(),
        command.stickyWarningBits());
  }

  private static RawCommandSnapshot withWarnings(
      RawCommandSnapshot command, int activeWarningBits, int stickyWarningBits) {
    return new RawCommandSnapshot(
        command.canId(),
        command.rawControlMode(),
        command.setpoint(),
        command.appliedOutput(),
        command.motorCurrentAmps(),
        command.velocity(),
        command.position(),
        command.busVoltage(),
        command.activeFaultBits(),
        command.stickyFaultBits(),
        activeWarningBits,
        stickyWarningBits);
  }

  private static void assertStopped(Response response, Status expectedStatus) {
    assertAll(
        () -> assertEquals(expectedStatus, response.status()),
        () -> assertEquals(0.0, response.appliedOutput()),
        () -> assertEquals(0.0, response.motorCurrentAmps()),
        () -> assertEquals(0.0, response.velocity()),
        () -> assertEquals(17.25, response.position()),
        () -> assertEquals(12.2, response.busVoltage()));
  }
}
