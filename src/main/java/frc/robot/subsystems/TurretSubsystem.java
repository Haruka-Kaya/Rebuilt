package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.constants.Constants.TurretConstants;
import frc.robot.constants.Constants.AprilTagConstants;
import frc.robot.utils.TalonFxContainer;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFxContainer m_motor = new TalonFxContainer(TurretConstants.TURRET_CAN_ID);

    private final VisionSubsystem m_vision;

    private double turret_kP;
    private double turret_kI;
    private double turret_kD;

    public boolean onTarget = false;

    public TurretSubsystem(VisionSubsystem vision) {
        this.m_vision = vision;
        
        m_motor.setBreakMode(true);

        m_motor.assignPIDValues(turret_kP, turret_kI, turret_kD);

        SmartDashboard.putNumber("Set turret_kP", 2.4);
        SmartDashboard.putNumber("Set turret_kI", 0);
        SmartDashboard.putNumber("Set turret_kD", 0.1);
    }

    public void setTurretAngle(double angleDegrees) {
        // convert turret degrees -> motor rotations before commanding
        m_motor.goToPostion(degreesToMotorRotations(angleDegrees));
    }

    private double degreesToMotorRotations(double degrees) {
        // turret degrees -> turret rotations -> motor rotations
        // turret rotations = degrees / 360
        // motor rotations = turret rotations * GEAR_RATIO
        return (degrees / 360.0) * TurretConstants.GEAR_RATIO;
    }

    public double getTurretAngle() {
        // motor rotations -> turret degrees
        double motorRotations = m_motor.motor.getPosition().getValueAsDouble();
        return motorRotationsToTurretDegrees(motorRotations);
    }

    private double motorRotationsToTurretDegrees(double motorRotations) {
        // turret rotations = motorRotations / GEAR_RATIO
        // degrees = turret rotations * 360
        return (motorRotations / TurretConstants.GEAR_RATIO) * 360.0;
    }
    
    public void autoAimWithLimelight() {
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
        turret_kP = SmartDashboard.getNumber("Set turret_kP", 2.4);
        turret_kI = SmartDashboard.getNumber("Set turret_kI", 0);
        turret_kP = SmartDashboard.getNumber("Set turret_kD", 0.1);

        m_motor.assignPIDValues(turret_kP, turret_kI, turret_kD); // remove in prod

        SmartDashboard.putNumber("Real turret angle", getTurretAngle());

        if(Telemetry.isHubActive()) {
            autoAimWithLimelight();
        }

        SmartDashboard.putBoolean("On target", onTarget);
    }
}
