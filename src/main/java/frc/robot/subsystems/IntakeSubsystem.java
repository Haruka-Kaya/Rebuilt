package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.utils.SparkMAXContainer;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMAXContainer m_intakeRoller = new SparkMAXContainer(IntakeConstants.INTAKE_ROLLER_CAN_ID);
    private final SparkMAXContainer m_actuatorMotor = new SparkMAXContainer(IntakeConstants.INTAKE_ACTUATOR_CAN_ID);

    private double actuator_kP;
    private double actuator_kI;
    private double actuator_kD;

    private double actuatorAngle;
    
    private double slurpPercent;
    private double spitPercent;


    public IntakeSubsystem() {
        m_intakeRoller.setBreakMode(false);
        m_actuatorMotor.setBreakMode(true);

        m_actuatorMotor.assignPIDValues(actuator_kP, actuator_kI, actuator_kD);
        m_actuatorMotor.motor.getEncoder().setPosition(0);

        SmartDashboard.putNumber("Set intake actuator_kP", 0.1);
        SmartDashboard.putNumber("Set intake actuator_kI", 0);
        SmartDashboard.putNumber("Set intake actuator_kD", 0);
        
        SmartDashboard.putNumber("Set intake actuator degrees", 0);

        SmartDashboard.putNumber("Set slurp roller percent", 0);
        SmartDashboard.putNumber("Set spit roller percent", 0);
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Real intake actuator degrees", m_actuatorMotor.getPosition() * 360);

        actuator_kP = SmartDashboard.getNumber("Set intake actuator_kP", 0.1);
        actuator_kI = SmartDashboard.getNumber("Set intake actuator_kI", 0);
        actuator_kP = SmartDashboard.getNumber("Set intake actuator_kD", 0);

        m_actuatorMotor.assignPIDValues(actuator_kP, actuator_kI, actuator_kD); // remove in prod

        actuatorAngle = SmartDashboard.getNumber("Set intake actuator degrees", 0);

        slurpPercent = SmartDashboard.getNumber("Set slurp roller percent", 0);
        spitPercent = SmartDashboard.getNumber("Set spit roller percent", 0);
    }
}