package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DiagnosticOutputSession.PulsePermit;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.utils.SparkMAXContainer;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMAXContainer leftMotor =
        new SparkMAXContainer(ClimberConstants.LEFT_MOTOR_CAN_ID);
    private final SparkMAXContainer rightMotor =
        new SparkMAXContainer(ClimberConstants.RIGHT_MOTOR_CAN_ID);
    private String lastBlockedMotion = "設計図待ち: direction/limit/homing未確定";

    public ClimberSubsystem() {
        leftMotor.setBreakMode(true);
        rightMotor.setBreakMode(true);
        leftMotor.setCurrentLimit(ClimberConstants.DIAGNOSTIC_CURRENT_LIMIT_AMPS);
        rightMotor.setCurrentLimit(ClimberConstants.DIAGNOSTIC_CURRENT_LIMIT_AMPS);

        SmartDashboard.putBoolean("Climber/Semantic Motion Configured", false);
    }

    /** Normal climb remains blocked until the design supplies direction, limits, and homing. */
    public boolean climb() {
        return rejectUnconfiguredMotion("climb");
    }

    public boolean extend() {
        return rejectUnconfiguredMotion("extend");
    }

    public boolean retract() {
        return rejectUnconfiguredMotion("retract");
    }

    /** Test-only raw polarity pulse; session ownership is enforced by the command layer. */
    public boolean runLeftUnhomedDiagnostic(double requestedDuty, PulsePermit permit) {
        return runUnhomedDiagnostic(leftMotor, rightMotor, requestedDuty, permit);
    }

    public boolean runRightUnhomedDiagnostic(double requestedDuty, PulsePermit permit) {
        return runUnhomedDiagnostic(rightMotor, leftMotor, requestedDuty, permit);
    }

    private boolean runUnhomedDiagnostic(
        SparkMAXContainer selectedMotor,
        SparkMAXContainer otherMotor,
        double requestedDuty,
        PulsePermit permit) {
        if (!DriverStation.isTestEnabled()
            || DriverStation.isFMSAttached()
            || !Double.isFinite(requestedDuty)
            || Math.abs(requestedDuty) <= 1e-9
            || Math.abs(requestedDuty) > ClimberConstants.DIAGNOSTIC_MAX_DUTY_CYCLE
            || permit == null
            || !permit.isValidFor(requestedDuty)
            || !isReady()) {
            stop();
            return false;
        }
        otherMotor.stop();
        boolean started = selectedMotor.setDutyCycleIfAuthorized(
            requestedDuty,
            () -> permit.isValidFor(requestedDuty)
                && DriverStation.isTestEnabled()
                && !DriverStation.isFMSAttached());
        if (!started) {
            stop();
        }
        return started;
    }

    public void stopUnhomedDiagnostic() {
        stop();
    }

    public void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }

    public boolean isReady() {
        return leftMotor.isReady() && rightMotor.isReady();
    }

    public String getDiagnosticStatus() {
        return "left=[" + leftMotor.getDiagnosticStatus() + "]"
            + " right=[" + rightMotor.getDiagnosticStatus() + "]";
    }

    private boolean rejectUnconfiguredMotion(String request) {
        lastBlockedMotion = request + ": direction/limit/homing未確定";
        stop();
        return false;
    }

    @Override
    public void periodic() {
        if (!DriverStation.isTestEnabled()) {
            stop();
        }
        SmartDashboard.putBoolean("Climber/Controllers Ready", isReady());
        SmartDashboard.putString("Climber/Last Blocked Motion", lastBlockedMotion);
        SmartDashboard.putString("Climber/Status", getDiagnosticStatus());
    }
}
