package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.utils.SparkMAXContainer;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMAXContainer actuatorMotor = new SparkMAXContainer(ShooterConstants.ACTUATOR_CAN_ID);
    private final SparkMAXContainer flywheelMotor_1 = new SparkMAXContainer(ShooterConstants.SHOOTER_1_CAN_ID);
    private final SparkMAXContainer flywheelMotor_2 = new SparkMAXContainer(ShooterConstants.SHOOTER_2_CAN_ID);

    private int flywheel_tolerance = 50;

    private double flywheelRPM = 500.0;

    private double flywheelkP = 0.1;
    private double flywheelkI = 0.0;
    private double flywheelkD = 0.0;

    public boolean flywheelIsSet = false;
    private boolean followerDiagnosticActive = false;

    
    private double actuatorPos = 5.0;

    private double actuatorkP = 0.1;
    private double actuatorkI = 0.0;
    private double actuatorkD = 0.0;

    public ShooterSubsystem() {
        if (actuatorMotor.isAvailable()) {
            actuatorMotor.motor.getEncoder().setPosition(0);
        }

        flywheelMotor_1.assignPIDValues(flywheelkP, flywheelkI, flywheelkD);
        flywheelMotor_2.setupAsFollowerMotor(flywheelMotor_1, true);
        actuatorMotor.assignPIDValues(actuatorkP, actuatorkI, actuatorkD);

        actuatorMotor.setBreakMode(true);
        actuatorMotor.setCurrentLimit(15);
        flywheelMotor_1.setCurrentLimit(30);
        flywheelMotor_2.setCurrentLimit(30);

        SmartDashboard.putNumber("Set flywheel_kP", 0.1);
        SmartDashboard.putNumber("Set flywheel_kI", 0);
        SmartDashboard.putNumber("Set flywheel_kD", 0);

        SmartDashboard.putNumber("Set flywheelRPM", 500);


        SmartDashboard.putNumber("Set shooter actuator_kP", 0.1);
        SmartDashboard.putNumber("Set shooter actuator_kI", 0);
        SmartDashboard.putNumber("Set shooter actuator_kD", 0);

        SmartDashboard.putNumber("Set shooter actuator degrees", 5);
    }

    /**
     * 
     * Set shooter speed based off network table values
     * 
     */
    public void setShooterSpeed() {
        flywheelMotor_1.setVelocity(flywheelRPM);
    }

    public void setActuatorAngle() {
        actuatorMotor.goToPostion(actuatorPos / 360);
    }

    public void setActuatorAngle(double degrees) {
        actuatorMotor.goToPostion(degrees / 360.0);
    }

    public void stop() {
        actuatorMotor.motor.stopMotor();
        flywheelMotor_1.motor.stopMotor();
        flywheelMotor_2.motor.stopMotor();
    }

    @Override
    public void periodic() {
        double requestedFlywheelkP = SmartDashboard.getNumber("Set flywheel_kP", 0.1);
        double requestedFlywheelkI = SmartDashboard.getNumber("Set flywheel_kI", 0);
        double requestedFlywheelkD = SmartDashboard.getNumber("Set flywheel_kD", 0);

        if (pidChanged(
                flywheelkP, flywheelkI, flywheelkD,
                requestedFlywheelkP, requestedFlywheelkI, requestedFlywheelkD)) {
            flywheelkP = requestedFlywheelkP;
            flywheelkI = requestedFlywheelkI;
            flywheelkD = requestedFlywheelkD;
            flywheelMotor_1.assignPIDValues(flywheelkP, flywheelkI, flywheelkD);
        }

        // Change to linear regresion line
        flywheelRPM = MathUtil.clamp(SmartDashboard.getNumber("Set flywheelRPM", 500), 0, 1000);


        double requestedActuatorkP = SmartDashboard.getNumber("Set shooter actuator_kP", 0.1);
        double requestedActuatorkI = SmartDashboard.getNumber("Set shooter actuator_kI", 0);
        double requestedActuatorkD = SmartDashboard.getNumber("Set shooter actuator_kD", 0);

        if (pidChanged(
                actuatorkP, actuatorkI, actuatorkD,
                requestedActuatorkP, requestedActuatorkI, requestedActuatorkD)) {
            actuatorkP = requestedActuatorkP;
            actuatorkI = requestedActuatorkI;
            actuatorkD = requestedActuatorkD;
            actuatorMotor.assignPIDValues(actuatorkP, actuatorkI, actuatorkD);
        }

        actuatorPos = MathUtil.clamp(
            SmartDashboard.getNumber("Set shooter actuator degrees", 5), 0, 5);

        SmartDashboard.putNumber("Real shooter acutator degrees", actuatorMotor.getPosition() * 360);
        SmartDashboard.putNumber("Real flywheelRPM", flywheelMotor_1.getVelocity());

        if(flywheelRPM < 3500) {
            flywheel_tolerance = 50;
        } else if(flywheelRPM < 4500) {
            flywheel_tolerance = 100;
        } else {
            flywheel_tolerance = 200;
        }

        flywheelIsSet = MathUtil.isNear(flywheelRPM, flywheelMotor_1.getVelocity(), flywheel_tolerance);

        SmartDashboard.putBoolean("Flywheel reved up", flywheelIsSet);
    }

    public void runFollowerDiagnostic() {
        if (flywheelMotor_2.isAvailable()) {
            if (!followerDiagnosticActive) {
                flywheelMotor_2.motor.pauseFollowerMode();
                followerDiagnosticActive = true;
            }
            flywheelMotor_2.motor.set(0.08);
        }
    }

    public void stopFollowerDiagnostic() {
        flywheelMotor_2.motor.stopMotor();
        if (followerDiagnosticActive) {
            flywheelMotor_2.motor.resumeFollowerMode();
        }
        followerDiagnosticActive = false;
    }

    public String getFollowerDiagnosticStatus() {
        return flywheelMotor_2.getDiagnosticStatus();
    }

    private static boolean pidChanged(
            double currentP, double currentI, double currentD,
            double requestedP, double requestedI, double requestedD) {
        return Double.compare(currentP, requestedP) != 0
                || Double.compare(currentI, requestedI) != 0
                || Double.compare(currentD, requestedD) != 0;
    }
}
