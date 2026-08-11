package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ManipulatorConstants;
import frc.robot.utils.SparkMAXContainer;

/**
 * Do not use directly. Access from the shooter instead
 */
public class ConveyorSubsystem extends SubsystemBase {
    private final SparkMAXContainer m_feederBelt = new SparkMAXContainer(ManipulatorConstants.CONVEYOR_CAN_ID);

    private double inPercent = 0.15;
    private double outPercent = -0.15;

    public ConveyorSubsystem() {
        m_feederBelt.setBreakMode(false);
        m_feederBelt.setCurrentLimit(15);

        SmartDashboard.putNumber("Set conveyer in percent", 0.15);
        SmartDashboard.putNumber("Set conveyer out percent", -0.15);
    }

    public boolean runConveyor() {
        return m_feederBelt.setDutyCycle(inPercent);
    }

    public void backfeedConveyor() {
        m_feederBelt.setDutyCycle(outPercent);
    }

    public void stop() {
        m_feederBelt.stop();
    }

    public boolean isReady() {
        return m_feederBelt.isReady();
    }

    @Override
    public void periodic() {
        inPercent = MathUtil.clamp(
            SmartDashboard.getNumber("Set conveyer in percent", 0.15), -0.20, 0.20);
        outPercent = MathUtil.clamp(
            SmartDashboard.getNumber("Set conveyer out percent", -0.15), -0.20, 0.20);
    }
}
