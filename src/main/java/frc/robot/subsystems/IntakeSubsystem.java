package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.utils.SparkMAXContainer;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMAXContainer m_intakeRoller = new SparkMAXContainer(IntakeConstants.INTAKE_ROLLER_CAN_ID);
    private final SparkMAXContainer m_actuatorMotor = new SparkMAXContainer(IntakeConstants.INTAKE_ACTUATOR_CAN_ID);

    private double actuator_kP = 0.1;
    private double actuator_kI = 0.0;
    private double actuator_kD = 0.0;

    private double actuatorAngle = 10.0;
    
    private double slurpPercent = 0.15;
    private double spitPercent = -0.15;


    public IntakeSubsystem() {
        m_intakeRoller.setBreakMode(false);
        m_actuatorMotor.setBreakMode(true);
        m_intakeRoller.setCurrentLimit(20);
        m_actuatorMotor.setCurrentLimit(15);

        m_actuatorMotor.assignPIDValues(actuator_kP, actuator_kI, actuator_kD);
        if (m_actuatorMotor.isAvailable()) {
            m_actuatorMotor.motor.getEncoder().setPosition(0);
        }

        SmartDashboard.putNumber("Set intake actuator_kP", 0.1);
        SmartDashboard.putNumber("Set intake actuator_kI", 0);
        SmartDashboard.putNumber("Set intake actuator_kD", 0);
        
        SmartDashboard.putNumber("Set intake actuator degrees", 10);

        SmartDashboard.putNumber("Set slurp roller percent", 0.15);
        SmartDashboard.putNumber("Set spit roller percent", -0.15);
    }

    private void slurp() {
        // m_intakeRoller.motor.set(IntakeConstants.ROLLER_IN_SPEED);
        m_intakeRoller.motor.set(slurpPercent);
        extendIntake();
    }

    private void spit() {
        // m_intakeRoller.motor.set(IntakeConstants.ROLLER_OUT_SPEED);
        m_intakeRoller.motor.set(spitPercent);
    }

    public void runIntake(boolean trueForIn) {
        if(trueForIn)
            slurp();
        else
            spit();
    }

    public void extendIntake() {
        // m_actuatorMotor.goToPostion(IntakeConstants.EXTENDED_ANGLE_DEGREES / 360);
        m_actuatorMotor.goToPostion(actuatorAngle / 360);
    }

    public void retractIntake() {
        m_actuatorMotor.goToPostion(0);
    }

    public void stop() {
        m_intakeRoller.motor.stopMotor();
    }

    public void stopAll() {
        m_intakeRoller.motor.stopMotor();
        m_actuatorMotor.motor.stopMotor();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Real intake actuator degrees", m_actuatorMotor.getPosition() * 360);

        double requestedActuatorKp = SmartDashboard.getNumber("Set intake actuator_kP", 0.1);
        double requestedActuatorKi = SmartDashboard.getNumber("Set intake actuator_kI", 0);
        double requestedActuatorKd = SmartDashboard.getNumber("Set intake actuator_kD", 0);

        if (Double.compare(actuator_kP, requestedActuatorKp) != 0
                || Double.compare(actuator_kI, requestedActuatorKi) != 0
                || Double.compare(actuator_kD, requestedActuatorKd) != 0) {
            actuator_kP = requestedActuatorKp;
            actuator_kI = requestedActuatorKi;
            actuator_kD = requestedActuatorKd;
            m_actuatorMotor.assignPIDValues(actuator_kP, actuator_kI, actuator_kD);
        }
        actuatorAngle = MathUtil.clamp(
            SmartDashboard.getNumber("Set intake actuator degrees", 10), 0, 10);

        slurpPercent = MathUtil.clamp(
            SmartDashboard.getNumber("Set slurp roller percent", 0.15), -0.20, 0.20);
        spitPercent = MathUtil.clamp(
            SmartDashboard.getNumber("Set spit roller percent", -0.15), -0.20, 0.20);
    }
}
