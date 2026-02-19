package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ManipulatorConstants;
import frc.robot.utils.SparkMAXContainer;

public class FeederSubsystem extends SubsystemBase {
    private final SparkMAXContainer m_feeder = new SparkMAXContainer(ManipulatorConstants.FEEDER_CAN_ID);

    private double feedPercent;
    private double rejectPercent;

    public FeederSubsystem() {
        m_feeder.setBreakMode(false);

        SmartDashboard.putNumber("Set feeder feed percent", 0);
        SmartDashboard.putNumber("Set feeder reject percent", 0);
    }

    public void feed() {
        // m_feeder.motor.set(ManipulatorConstants.FEEDER_IN_SPEED);
        m_feeder.motor.set(feedPercent);
    }

    public void reject() {
        // m_feeder.motor.set(ManipulatorConstants.FEEDER_OUT_SPEED);
        m_feeder.motor.set(rejectPercent);
    }

    public void stop() {
        m_feeder.motor.stopMotor();
    }

    @Override
    public void periodic() {
        feedPercent = SmartDashboard.getNumber("Set feeder feed percent", 0);
        rejectPercent = SmartDashboard.getNumber("Set feeder reject percent", 0);
    }
}