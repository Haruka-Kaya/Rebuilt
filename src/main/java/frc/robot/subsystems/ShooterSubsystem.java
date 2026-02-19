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

    private double flywheelRPM;

    private double flywheelkP;
    private double flywheelkI;
    private double flywheelkD;

    public boolean flywheelIsSet = false;

    
    private double actuatorPos;

    private double actuatorkP;
    private double actuatorkI;
    private double actuatorkD;

    public ShooterSubsystem() {
        actuatorMotor.motor.getEncoder().setPosition(0);

        flywheelMotor_1.assignPIDValues(flywheelkP, flywheelkI, flywheelkD);
        flywheelMotor_2.setupAsFollowerMotor(flywheelMotor_1, true);
        actuatorMotor.assignPIDValues(actuatorkP, actuatorkI, actuatorkD);

        actuatorMotor.setBreakMode(true);

        SmartDashboard.putNumber("Set flywheel_kP", 0.1);
        SmartDashboard.putNumber("Set flywheel_kI", 0);
        SmartDashboard.putNumber("Set flywheel_kD", 0);

        SmartDashboard.putNumber("Set flywheelRPM", 0);


        SmartDashboard.putNumber("Set shooter actuator_kP", 0.1);
        SmartDashboard.putNumber("Set shooter actuator_kI", 0);
        SmartDashboard.putNumber("Set shooter actuator_kD", 0);

        SmartDashboard.putNumber("Set shooter actuator degrees", 0);
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
        actuatorMotor.goToPostion(degrees);
    }

    public void stop() {
        actuatorMotor.motor.stopMotor();
        flywheelMotor_1.motor.stopMotor();
        flywheelMotor_2.motor.stopMotor();
    }

    @Override
    public void periodic() {
        flywheelkP = SmartDashboard.getNumber("Set flywheel_kP", 0.1);
        flywheelkI = SmartDashboard.getNumber("Set flywheel_kI", 0);
        flywheelkD = SmartDashboard.getNumber("Set flywheel_kD", 0);

        flywheelMotor_1.assignPIDValues(flywheelkP, flywheelkI, flywheelkD); // remove in prod

        // Change to linear regresion line
        flywheelRPM = SmartDashboard.getNumber("Set flywheelRPM", 0);


        actuatorkP = SmartDashboard.getNumber("Set shooter actuator_kP", 0.1);
        actuatorkI = SmartDashboard.getNumber("Set shooter actuator_kI", 0);
        actuatorkD = SmartDashboard.getNumber("Set shooter actuator_kD", 0);

        actuatorPos = SmartDashboard.getNumber("Set shooter actuator degrees", 0) / 360;

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
}