// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /** Temporary safety switches used while diagnosing the robot on the bench. */
  public static final class DebugConstants {
    /** Enable drivetrain output for the current full-system test. */
    public static final boolean ALLOW_SWERVE_OUTPUT = true;

    /** Maximum commanded translation while the robot is being diagnosed. */
    public static final double MAX_SWERVE_TRANSLATION_FRACTION = 0.05;

    /** Maximum commanded rotation while the robot is being diagnosed. */
    public static final double MAX_SWERVE_ROTATION_RADIANS_PER_SECOND = Math.PI * 0.10;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;

    public static final int kOperatorControllerPort = 1;

    public static final int kMaintenanceControllerPort = 2;
  }

  public static final class AutoConstants {
    /**
     * Keep PathPlanner motion disabled until wheel radius, module locations, gearing, and maximum
     * speed are reconciled with the real robot/CAD. The current generated Tuner constants and
     * PathPlanner settings disagree, so enabling an autonomous path would be an unverified motion.
     */
    public static final boolean CALIBRATED_AUTONOMOUS_ENABLED = false;

    public static final String CALIBRATION_BLOCK_REASON =
        "BLOCKED: verify swerve wheel radius, module geometry, gearing, and max speed";

    public static final double kMaxSpeedMetersPerSecond = 0.45;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.toRadians(15.0);
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.toRadians(30.0);

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_ACTUATOR_CAN_ID = 30;
    public static final int INTAKE_ROLLER_CAN_ID = 31;
    public static final int ROLLER_CURRENT_LIMIT_AMPS = 20;

    public static final double EXTENDED_ANGLE_DEGREES = 75;

    // Motor speeds from -1 to +1
    public static final double ROLLER_IN_SPEED = 0.75;
    public static final double ROLLER_OUT_SPEED = -0.75;
  }

  public static final class ManipulatorConstants {
    public static final int FEEDER_CAN_ID = 32;
    public static final int CONVEYOR_CAN_ID = 33;
    public static final int FEEDER_CURRENT_LIMIT_AMPS = 10;
    public static final int CONVEYOR_CURRENT_LIMIT_AMPS = 15;

    /**
     * Keep ID 32 motion blocked until the 2026-08-10 44.14 A / approximately 0 rpm
     * stall is physically cleared and a controlled retest passes.
     */
    public static final boolean FEEDER_MOTION_BLOCKED_KNOWN_STALL = true;

    /**
     * Enable only after the physical jam/power branch is inspected. This permits the guarded 3%
     * Hardware Self-Test stage while normal feed/reject remains blocked by the flag above.
     */
    public static final boolean FEEDER_CONTROLLED_RETEST_ENABLED = false;

    // Motor speeds from -1 to +1
    public static final double CONVEYOR_IN_SPEED = 0.75;
    public static final double CONVEYOR_OUT_SPEED = -0.75;

    public static final double FEEDER_IN_SPEED = 0.75;
    public static final double FEEDER_OUT_SPEED = -0.75;
  }

  public static final class ClimberConstants {
    public static final int LEFT_MOTOR_CAN_ID = 34;
    public static final int RIGHT_MOTOR_CAN_ID = 35;

    /** Conservative limits until the motor type, gearing, and load are verified from the design. */
    public static final int DIAGNOSTIC_CURRENT_LIMIT_AMPS = 10;
    public static final double DIAGNOSTIC_MAX_DUTY_CYCLE = 0.03;
    public static final double DIAGNOSTIC_PULSE_SECONDS = 0.35;
  }

  public static final class ShooterConstants {
    public static final int SHOOTER_1_CAN_ID = 36;
    public static final int SHOOTER_2_CAN_ID = 37;
    public static final int ACTUATOR_CAN_ID = 38;
    public static final int FLYWHEEL_CURRENT_LIMIT_AMPS = 30;
  }

  public static final class HardwareTestConstants {
    public static final double ARM_LIFETIME_SECONDS = 15.0;
    public static final double OPEN_LOOP_DUTY_CYCLE = 0.03;
    public static final double OPEN_LOOP_STAGE_SECONDS = 0.40;
    public static final double STOP_CONFIRM_TIMEOUT_SECONDS = 2.0;
    public static final double MAX_STOPPED_SWERVE_SPEED_METERS_PER_SECOND = 0.01;
    public static final double MIN_SWERVE_MODULE_SPEED_METERS_PER_SECOND = 0.02;
    public static final double MAX_SWERVE_VECTOR_ERROR_DEGREES = 20.0;
    public static final double MAX_SWERVE_DIAGNOSTIC_DRIVE_CURRENT_AMPS = 40.0;
    public static final double MAX_SWERVE_DIAGNOSTIC_STEER_CURRENT_AMPS = 30.0;
  }

  public static final class LimelightConstants {
    public static final String TURRET_LIMELIGHT_NAME = "TURRET_EYES";
    public static final String DRIVE_LIMELIGHT_NAME = "DRIVE_EYES";

    public static final double MOUNT_ANGLE_DEG = 0.0;
    public static final double MOUNT_HEIGHT_METERS = 0.43;

    public static final int PIPELINE_FUEL = 0;
    public static final int PIPELINE_APRILTAG = 1;

    public static final Matrix<N3, N1> VISION_STD_DEVS =
        VecBuilder.fill(
                0.7,                // x meters
                0.7,                // y meters
                Math.toRadians(10)  // theta radians
        );
  }

  public static final class AprilTagConstants {
    public static final int[] VALID_RED_HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};
    public static final int[] VALID_BLUE_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};
  }

  public static final class TurretConstants {
    public static final int TURRET_CAN_ID = 39;

    // Gear ratio motor : turret
    public static final double GEAR_RATIO = 36.0;
    
    // Auto-aim tuning
    public static final double AIM_DEADBAND_DEG = 0.5; // don't react to tiny offsets
    public static final double SAFE_KP = 1.0;          // multiplier for tx -> turret degrees
    public static final double MAX_AIM_STEP_DEGREES = 5.0;
    public static final double MAX_CLOSED_LOOP_OUTPUT = 0.15;
    public static final int REQUIRED_ON_TARGET_FRAMES = 2;

    // Mechanical limits — set to your real stops
    public static final double MIN_ANGLE_DEGREES = -180.0;
    public static final double MAX_ANGLE_DEGREES = 180.0;
  }
}
