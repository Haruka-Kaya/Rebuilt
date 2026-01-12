package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax m_intakeRoller = new SparkMax(IntakeConstants.INTAKE_ROLLER_CAN_ID, MotorType.kBrushless);
    private final SparkMax m_intakeBelt = new SparkMax(IntakeConstants.INTAKE_BELT_CAN_ID, MotorType.kBrushless);

    public IntakeSubsystem() {

    }

    private void slurp() {
        m_intakeRoller.set(IntakeConstants.INTAKE_ROLLER_IN_SPEED);
        m_intakeBelt.set(IntakeConstants.INTAKE_BELT_IN_SPEED);
    }

    private void spit() {
        m_intakeRoller.set(IntakeConstants.INTAKE_ROLLER_OUT_SPEED);
        m_intakeBelt.set(IntakeConstants.INTAKE_BELT_OUT_SPEED);
    }

    public void runIntake(boolean trueForIn) {
        if(trueForIn)
            slurp();
        else
            spit();
    }

    public void stop() {
        m_intakeBelt.set(0);
        m_intakeRoller.set(0);
    }
}
