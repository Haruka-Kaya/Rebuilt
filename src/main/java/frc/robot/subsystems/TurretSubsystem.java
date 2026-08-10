package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.constants.Constants.TurretConstants;
import frc.robot.constants.Constants.AprilTagConstants;
import frc.robot.utils.SparkMAXContainer;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMAXContainer m_motor = new SparkMAXContainer(TurretConstants.TURRET_CAN_ID);

    private final VisionSubsystem m_vision;

    private double turret_kP = 2.4;
    private double turret_kI = 0.0;
    private double turret_kD = 0.1;
    private boolean motorConfigured = false;

    public boolean onTarget = false;

    public TurretSubsystem(VisionSubsystem vision) {
        this.m_vision = vision;
        
        SmartDashboard.putNumber("Set turret_kP", 2.4);
        SmartDashboard.putNumber("Set turret_kI", 0);
        SmartDashboard.putNumber("Set turret_kD", 0.1);
    }

    public void setTurretAngle(double angleDegrees) {
        if (!m_motor.isAvailable()) return;
        // convert turret degrees -> motor rotations before commanding
        m_motor.goToPostion(degreesToMotorRotations(angleDegrees));
    }

    public void stop() {
        if (m_motor.isAvailable()) {
            m_motor.motor.stopMotor();
        }
        onTarget = false;
    }

    private double degreesToMotorRotations(double degrees) {
        // turret degrees -> turret rotations -> motor rotations
        // turret rotations = degrees / 360
        // motor rotations = turret rotations * GEAR_RATIO
        return (degrees / 360.0) * TurretConstants.GEAR_RATIO;
    }

    public double getTurretAngle() {
        if (!m_motor.isAvailable()) return 0;
        // motor rotations -> turret degrees
        double motorRotations = m_motor.getPosition();
        return motorRotationsToTurretDegrees(motorRotations);
    }

    private double motorRotationsToTurretDegrees(double motorRotations) {
        // turret rotations = motorRotations / GEAR_RATIO
        // degrees = turret rotations * 360
        return (motorRotations / TurretConstants.GEAR_RATIO) * 360.0;
    }
    
    public void autoAimWithLimelight() {
        if (!m_motor.isAvailable()) {
            onTarget = false;
            return;
        }
        // only act if target valid
        if (!m_vision.hasTarget()) {
            onTarget = false;
            return;
        }

        boolean nope = true;
        
        if(Telemetry.isRedAlliance()) {
            for(int i = 0; i < 6; i++) {
                if(m_vision.isTrackingTag(AprilTagConstants.VALID_RED_HUB_TAG_IDS[i])) {
                    nope = false;
                    break;
                }
            }
            if(nope == true) {
                onTarget = false;
                return;
            }
        } else {
            for(int i = 0; i < 6; i++) {
                if(m_vision.isTrackingTag(AprilTagConstants.VALID_BLUE_HUB_TAG_IDS[i])) {
                    nope = false;
                    break;
                }
            }
            if(nope == true) {
                onTarget = false;
                return;
            }
        }

        double tx = m_vision.getTx(); // degrees offset (crosshair -> target)
        if (Math.abs(tx) < TurretConstants.AIM_DEADBAND_DEG) {
            onTarget = true;
            return;
        }

        onTarget = false;

        // compute new turret setpoint: add camera offset to current turret angle
        double currentAngle = getTurretAngle();
        double commandedAngle = currentAngle - tx;

        // clamp to mechanical limits
        commandedAngle = MathUtil.clamp(commandedAngle, TurretConstants.MIN_ANGLE_DEGREES, TurretConstants.MAX_ANGLE_DEGREES);

        setTurretAngle(commandedAngle);
    }

    @Override
    public void periodic() {
        boolean motorConnected = m_motor.isAvailable();
        SmartDashboard.putBoolean("Turret motor connected", motorConnected);
        if (!motorConnected) {
            motorConfigured = false;
            onTarget = false;
            SmartDashboard.putBoolean("On target", false);
            return;
        }

        if (!motorConfigured) {
            m_motor.setBreakMode(true);
            m_motor.setCurrentLimit(15);
            m_motor.assignPIDValues(turret_kP, turret_kI, turret_kD);
            motorConfigured = true;
        }

        double requestedTurretKp = SmartDashboard.getNumber("Set turret_kP", 2.4);
        double requestedTurretKi = SmartDashboard.getNumber("Set turret_kI", 0);
        double requestedTurretKd = SmartDashboard.getNumber("Set turret_kD", 0.1);

        if (Double.compare(turret_kP, requestedTurretKp) != 0
                || Double.compare(turret_kI, requestedTurretKi) != 0
                || Double.compare(turret_kD, requestedTurretKd) != 0) {
            turret_kP = requestedTurretKp;
            turret_kI = requestedTurretKi;
            turret_kD = requestedTurretKd;
            m_motor.assignPIDValues(turret_kP, turret_kI, turret_kD);
        }

        SmartDashboard.putNumber("Real turret angle", getTurretAngle());

        SmartDashboard.putBoolean("On target", onTarget);
    }
}
