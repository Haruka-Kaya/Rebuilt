package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ManipulatorConstants;
import frc.robot.utils.DashboardApplyGate;
import frc.robot.utils.DashboardApplyGate.Decision;
import frc.robot.utils.SparkMAXContainer;

public class FeederSubsystem extends SubsystemBase {
    private static final String TUNING_APPLY_KEY = "Tuning/Feeder/Apply";
    private static final String TUNING_STATUS_KEY = "Tuning/Feeder/Status";
    private static final double MAX_DUTY_CYCLE = 0.20;

    private final SparkMAXContainer m_feeder = new SparkMAXContainer(ManipulatorConstants.FEEDER_CAN_ID);
    private final DashboardApplyGate tuningApplyGate = new DashboardApplyGate();

    private double feedPercent = 0.15;
    private double rejectPercent = -0.15;

    public FeederSubsystem() {
        m_feeder.setBreakMode(false);
        m_feeder.setCurrentLimit(ManipulatorConstants.FEEDER_CURRENT_LIMIT_AMPS);

        SmartDashboard.putNumber("Set feeder feed percent", 0.15);
        SmartDashboard.putNumber("Set feeder reject percent", -0.15);
        SmartDashboard.putBoolean(TUNING_APPLY_KEY, false);
        SmartDashboard.putString(TUNING_STATUS_KEY, "ACTIVE_DEFAULTS");
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

    public String getDiagnosticStatus() {
        return m_feeder.getDiagnosticStatus();
    }

    @Override
    public void periodic() {
        Decision decision = tuningApplyGate.poll(TUNING_APPLY_KEY);
        if (decision == Decision.NONE) {
            return;
        }
        if (decision != Decision.APPLY
                || !DriverStation.isDisabled()
                || DriverStation.isFMSAttached()) {
            publishActiveTuning();
            SmartDashboard.putString(TUNING_STATUS_KEY, "REJECTED_" + decision.name());
            return;
        }

        double requestedFeed = SmartDashboard.getNumber("Set feeder feed percent", feedPercent);
        double requestedReject = SmartDashboard.getNumber(
            "Set feeder reject percent", rejectPercent);
        if (!DashboardApplyGate.allFiniteInRange(
                new double[] {requestedFeed, requestedReject},
                new double[] {0.0, -MAX_DUTY_CYCLE},
                new double[] {MAX_DUTY_CYCLE, 0.0})) {
            publishActiveTuning();
            SmartDashboard.putString(TUNING_STATUS_KEY, "REJECTED_INVALID_OR_WRONG_SIGN");
            return;
        }
        feedPercent = requestedFeed;
        rejectPercent = requestedReject;
        SmartDashboard.putString(TUNING_STATUS_KEY, "QUEUED_DISABLED");
    }

    private void publishActiveTuning() {
        SmartDashboard.putNumber("Set feeder feed percent", feedPercent);
        SmartDashboard.putNumber("Set feeder reject percent", rejectPercent);
    }
}
