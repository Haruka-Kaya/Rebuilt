package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem extends SubsystemBase {
    private final SparkMax m_feederBelt = new SparkMax(ManipulatorConstants.FEEDER_BELT_CAN_ID, MotorType.kBrushless);
    private final SparkMax m_feederTube = new SparkMax(ManipulatorConstants.FEEDER_TUBE_CAN_ID, MotorType.kBrushless);

    public ManipulatorSubsystem() {

    }

    public void runManipulator() {
        m_feederBelt.set(ManipulatorConstants.FEEDER_BELT_SPEED);
        m_feederTube.set(ManipulatorConstants.FEEDER_TUBE_SPEED);
    }

    public void stop() {
        m_feederBelt.set(0);
        m_feederTube.set(0);
    }
}
