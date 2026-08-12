package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DiagnosticOutputSession.PulsePermit;
import frc.robot.constants.Constants.ManipulatorConstants;
import frc.robot.constants.Constants.HardwareTestConstants;
import frc.robot.utils.DashboardApplyGate;
import frc.robot.utils.DashboardApplyGate.Decision;
import frc.robot.utils.SparkMAXContainer;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.Snapshot;

/**
 * Do not use directly. Access from the shooter instead
 */
public class ConveyorSubsystem extends SubsystemBase {
    private static final String TUNING_APPLY_KEY = "Tuning/Conveyor/Apply";
    private static final String TUNING_STATUS_KEY = "Tuning/Conveyor/Status";
    private static final double MAX_DUTY_CYCLE = 0.20;

    private final SparkMAXContainer m_feederBelt = new SparkMAXContainer(ManipulatorConstants.CONVEYOR_CAN_ID);
    private final DashboardApplyGate tuningApplyGate = new DashboardApplyGate();

    private double inPercent = 0.15;
    private double outPercent = -0.15;

    public ConveyorSubsystem() {
        m_feederBelt.setBreakMode(false);
        m_feederBelt.setCurrentLimit(ManipulatorConstants.CONVEYOR_CURRENT_LIMIT_AMPS);

        SmartDashboard.putNumber("Set conveyer in percent", 0.15);
        SmartDashboard.putNumber("Set conveyer out percent", -0.15);
        SmartDashboard.putBoolean(TUNING_APPLY_KEY, false);
        SmartDashboard.putString(TUNING_STATUS_KEY, "ACTIVE_DEFAULTS");
    }

    public boolean runConveyor() {
        return m_feederBelt.setDutyCycle(inPercent);
    }

    public boolean backfeedConveyor() {
        return m_feederBelt.setDutyCycle(outPercent);
    }

    public void stop() {
        m_feederBelt.stop();
    }

    public boolean isReady() {
        return m_feederBelt.isReady();
    }

    public boolean runDiagnostic(double requestedDuty, PulsePermit permit) {
        if (!DriverStation.isTestEnabled()
                || DriverStation.isFMSAttached()
                || !Double.isFinite(requestedDuty)
                || Math.abs(requestedDuty) > HardwareTestConstants.OPEN_LOOP_DUTY_CYCLE
                || permit == null
                || !permit.isValidFor(requestedDuty)) {
            stop();
            return false;
        }
        return m_feederBelt.setDutyCycleIfAuthorized(
            requestedDuty,
            () -> permit.isValidFor(requestedDuty)
                && DriverStation.isTestEnabled()
                && !DriverStation.isFMSAttached());
    }

    public Snapshot getDiagnosticSnapshot(boolean commandAccepted) {
        return m_feederBelt.getDiagnosticSnapshot(commandAccepted);
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

        double requestedIn = SmartDashboard.getNumber("Set conveyer in percent", inPercent);
        double requestedOut = SmartDashboard.getNumber("Set conveyer out percent", outPercent);
        if (!DashboardApplyGate.allFiniteInRange(
                new double[] {requestedIn, requestedOut},
                new double[] {0.0, -MAX_DUTY_CYCLE},
                new double[] {MAX_DUTY_CYCLE, 0.0})) {
            publishActiveTuning();
            SmartDashboard.putString(TUNING_STATUS_KEY, "REJECTED_INVALID_OR_WRONG_SIGN");
            return;
        }
        inPercent = requestedIn;
        outPercent = requestedOut;
        SmartDashboard.putString(TUNING_STATUS_KEY, "QUEUED_DISABLED");
    }

    private void publishActiveTuning() {
        SmartDashboard.putNumber("Set conveyer in percent", inPercent);
        SmartDashboard.putNumber("Set conveyer out percent", outPercent);
    }
}
