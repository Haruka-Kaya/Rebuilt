package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ManipulatorConstants;
import frc.robot.utils.SparkMAXContainer;

public class FeederSubsystem extends SubsystemBase {
    private final SparkMAXContainer m_feeder = new SparkMAXContainer(ManipulatorConstants.FEEDER_CAN_ID);

    private double feedPercent = 0.15;
    private double rejectPercent = -0.15;

    public FeederSubsystem() {
        m_feeder.setBreakMode(false);
        m_feeder.setCurrentLimit(10);

        SmartDashboard.putNumber("Set feeder feed percent", 0.15);
        SmartDashboard.putNumber("Set feeder reject percent", -0.15);
    }

    public boolean feed() {
        return m_feeder.setDutyCycle(feedPercent);
    }

    public void reject() {
        m_feeder.setDutyCycle(rejectPercent);
    }

    public void stop() {
        m_feeder.stop();
    }

    public boolean isReady() {
        return m_feeder.isReady();
    }

    public void runDiagnostic() {
        m_feeder.setDutyCycle(0.03);
    }

    public String getDiagnosticStatus() {
        return m_feeder.getDiagnosticStatus();
    }

    @Override
    public void periodic() {
        feedPercent = MathUtil.clamp(
            SmartDashboard.getNumber("Set feeder feed percent", 0.15), -0.20, 0.20);
        rejectPercent = MathUtil.clamp(
            SmartDashboard.getNumber("Set feeder reject percent", -0.15), -0.20, 0.20);
    }
}
