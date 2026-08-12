package frc.robot.utils;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkSimFaultManager;
import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import java.util.Objects;
import java.util.function.Function;

/**
 * Narrow simulation-only façade for one SPARK MAX.
 *
 * <p>This deliberately exposes neither the mutable vendor controller nor a physics {@code
 * iterate()} API. The robot's motor type, gearing, inertia, and mechanism limits are not verified,
 * so callers may only inject raw telemetry and the two fault conditions needed by recovery tests.
 * Raw position injection never establishes a mechanism reference.
 */
public final class SparkSimulationHandle {
  private static final int CAN_FAULT_MASK = 1 << 3;
  private static final int RESET_WARNING_MASK = 1 << 6;

  private final SparkMax motor;
  private final SparkMaxSim simulation;
  private final SparkSimFaultManager faultManager;
  private final SimInt rawControlMode;
  private final Object ioLock;

  SparkSimulationHandle(SparkMax motor, Object ioLock) {
    this.motor = Objects.requireNonNull(motor, "motor");
    this.ioLock = Objects.requireNonNull(ioLock, "ioLock");
    // This motor model is a non-physical placeholder because iterate() is intentionally hidden.
    simulation = new SparkMaxSim(motor, DCMotor.getNEO(1));
    faultManager = simulation.getFaultManager();
    rawControlMode = Objects.requireNonNull(
        new SimDeviceSim("SPARK MAX [" + motor.getDeviceId() + "]").getInt("Control Mode"),
        "REV simulation Control Mode");
  }

  public int canId() {
    return motor.getDeviceId();
  }

  /** Returns one coherent observation relative to raw telemetry injection and setpoint writes. */
  public SimulationSnapshot observe() {
    synchronized (ioLock) {
      return new SimulationSnapshot(
          canId(),
          simulation.getAppliedOutput(),
          simulation.getMotorCurrent(),
          simulation.getVelocity(),
          simulation.getPosition(),
          simulation.getBusVoltage(),
          simulation.getSetpoint(),
          rawControlMode.get(),
          motor.getFaults().rawBits,
          motor.getStickyFaults().rawBits,
          motor.getWarnings().rawBits,
          motor.getStickyWarnings().rawBits);
    }
  }

  /**
   * Atomically observes a raw command and applies a pure command-echo response.
   *
   * <p>Package-private by design: the production controller remains the only setpoint authority,
   * and the simulation coordinator cannot acquire the vendor I/O lock between observation and
   * response application.
   */
  SparkRawCommandEchoSimulation.Response applyRawCommandEcho(
      Function<RawCommandSnapshot, SparkRawCommandEchoSimulation.Response> responseFactory) {
    Objects.requireNonNull(responseFactory, "responseFactory");
    synchronized (ioLock) {
      RawCommandSnapshot command = rawCommandSnapshotLocked();
      SparkRawCommandEchoSimulation.Response response =
          Objects.requireNonNull(responseFactory.apply(command), "response");
      if (response.canId() != command.canId()) {
        throw new IllegalArgumentException("response CAN ID does not match simulation handle");
      }

      simulation.setAppliedOutput(response.appliedOutput());
      simulation.setMotorCurrent(response.motorCurrentAmps());
      simulation.setVelocity(response.velocity());
      simulation.setPosition(response.position());
      simulation.getRelativeEncoderSim().setVelocity(response.velocity());
      simulation.getRelativeEncoderSim().setPosition(response.position());
      simulation.setBusVoltage(response.busVoltage());
      return response;
    }
  }

  private RawCommandSnapshot rawCommandSnapshotLocked() {
    return new RawCommandSnapshot(
        canId(),
        rawControlMode.get(),
        simulation.getSetpoint(),
        simulation.getAppliedOutput(),
        simulation.getMotorCurrent(),
        simulation.getVelocity(),
        simulation.getPosition(),
        simulation.getBusVoltage(),
        motor.getFaults().rawBits,
        motor.getStickyFaults().rawBits,
        motor.getWarnings().rawBits,
        motor.getStickyWarnings().rawBits);
  }

  /**
   * Atomically replaces the raw simulation telemetry used by REV periodic status frames.
   *
   * <p>Values are intentionally not sanitized: NaN and out-of-range telemetry are required to
   * prove that the production validation path fails closed.
   */
  public void injectRawTelemetry(RawTelemetry telemetry) {
    Objects.requireNonNull(telemetry, "telemetry");
    synchronized (ioLock) {
      simulation.setAppliedOutput(telemetry.appliedOutput());
      simulation.setMotorCurrent(telemetry.motorCurrentAmps());
      simulation.setVelocity(telemetry.velocity());
      simulation.setPosition(telemetry.position());
      // PeriodicStatus2 is sourced from the selected relative encoder SimDevice. REV's raw
      // SparkSim setters do not mirror into that separate device unless iterate() is used.
      simulation.getRelativeEncoderSim().setVelocity(telemetry.velocity());
      simulation.getRelativeEncoderSim().setPosition(telemetry.position());
      simulation.setBusVoltage(telemetry.busVoltage());
    }
  }

  /** Injects REV's CAN-fault status bit. This is not a simulation of a disconnected controller. */
  public void setCanFault(boolean active, boolean sticky) {
    synchronized (ioLock) {
      Faults activeFaults = motor.getFaults();
      Faults stickyFaults = motor.getStickyFaults();
      faultManager.setFaults(new Faults(updateBit(activeFaults.rawBits, CAN_FAULT_MASK, active)));
      faultManager.setStickyFaults(
          new Faults(updateBit(stickyFaults.rawBits, CAN_FAULT_MASK, sticky)));
    }
  }

  /** Injects REV's controller-reset warning without altering unrelated warning bits. */
  public void setResetWarning(boolean active, boolean sticky) {
    synchronized (ioLock) {
      Warnings activeWarnings = motor.getWarnings();
      Warnings stickyWarnings = motor.getStickyWarnings();
      faultManager.setWarnings(
          new Warnings(updateBit(activeWarnings.rawBits, RESET_WARNING_MASK, active)));
      faultManager.setStickyWarnings(
          new Warnings(updateBit(stickyWarnings.rawBits, RESET_WARNING_MASK, sticky)));
    }
  }

  /** Releases the REV fault-manager SimDevice that SparkMax.close() leaves allocated. */
  void closeSimulationResourcesForTesting() {
    synchronized (ioLock) {
      int faultManagerHandle = SimDeviceDataJNI.getSimDeviceHandle(
          "SPARK MAX [" + canId() + "] FAULT MANAGER");
      if (faultManagerHandle > 0) {
        SimDeviceJNI.freeSimDevice(faultManagerHandle);
      }
    }
  }

  private static int updateBit(int value, int mask, boolean set) {
    return set ? value | mask : value & ~mask;
  }

  public record RawTelemetry(
      double appliedOutput,
      double motorCurrentAmps,
      double velocity,
      double position,
      double busVoltage) {}

  public record SimulationSnapshot(
      int canId,
      double appliedOutput,
      double motorCurrentAmps,
      double velocity,
      double position,
      double busVoltage,
      double setpoint,
      int rawControlMode,
      int activeFaultBits,
      int stickyFaultBits,
      int activeWarningBits,
      int stickyWarningBits) {}

  /** Immutable raw REV command/status view used by the non-physical command echo. */
  public record RawCommandSnapshot(
      int canId,
      int rawControlMode,
      double setpoint,
      double appliedOutput,
      double motorCurrentAmps,
      double velocity,
      double position,
      double busVoltage,
      int activeFaultBits,
      int stickyFaultBits,
      int activeWarningBits,
      int stickyWarningBits) {}
}
