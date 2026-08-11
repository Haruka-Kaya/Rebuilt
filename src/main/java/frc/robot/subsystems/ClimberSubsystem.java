package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberDiagnosticLatch.MotorSide;
import frc.robot.utils.SparkMAXContainer;

public class ClimberSubsystem extends SubsystemBase {
    public static final String DIAGNOSTIC_ARM_KEY = "Climber Diagnostic/Armed";
    public static final String MOTOR_TYPE_VERIFIED_KEY =
        "Climber Diagnostic/Brushless Motor Type Verified";

    private final SparkMAXContainer leftMotor =
        new SparkMAXContainer(ClimberConstants.LEFT_MOTOR_CAN_ID);
    private final SparkMAXContainer rightMotor =
        new SparkMAXContainer(ClimberConstants.RIGHT_MOTOR_CAN_ID);
    private final ClimberDiagnosticLatch diagnosticLatch = new ClimberDiagnosticLatch();
    private String lastBlockedMotion = "設計図待ち: direction/limit/homing未確定";

    public ClimberSubsystem() {
        leftMotor.setBreakMode(true);
        rightMotor.setBreakMode(true);
        leftMotor.setCurrentLimit(ClimberConstants.DIAGNOSTIC_CURRENT_LIMIT_AMPS);
        rightMotor.setCurrentLimit(ClimberConstants.DIAGNOSTIC_CURRENT_LIMIT_AMPS);

        SmartDashboard.putBoolean(DIAGNOSTIC_ARM_KEY, false);
        SmartDashboard.putBoolean(MOTOR_TYPE_VERIFIED_KEY, false);
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

    /** Test-only, armed, one-motor polarity diagnostic. */
    public boolean runDiagnostic(MotorSide side, double requestedDuty) {
        boolean ready = isReady();
        var accepted = diagnosticLatch.accept(
            side,
            requestedDuty,
            ClimberConstants.DIAGNOSTIC_MAX_DUTY_CYCLE,
            DriverStation.isTestEnabled(),
            DriverStation.isFMSAttached(),
            SmartDashboard.getBoolean(DIAGNOSTIC_ARM_KEY, false)
                && SmartDashboard.getBoolean(MOTOR_TYPE_VERIFIED_KEY, false),
            ready);
        if (accepted.isEmpty()) {
            stop();
            return false;
        }

        double duty = accepted.getAsDouble();
        boolean started;
        if (side == MotorSide.LEFT) {
            rightMotor.stop();
            started = leftMotor.setDutyCycle(duty);
        } else {
            leftMotor.stop();
            started = rightMotor.setDutyCycle(duty);
        }
        if (!started) {
            stop();
        }
        return started;
    }

    /** Snapshotted by the command at the start of each diagnostic gesture. */
    public boolean canStartDiagnostic() {
        return DriverStation.isTestEnabled()
            && !DriverStation.isFMSAttached()
            && SmartDashboard.getBoolean(DIAGNOSTIC_ARM_KEY, false)
            && SmartDashboard.getBoolean(MOTOR_TYPE_VERIFIED_KEY, false)
            && isReady();
    }

    public void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }

    public boolean isReady() {
        return leftMotor.isReady() && rightMotor.isReady();
    }

    public String getDiagnosticStatus() {
        return "selection=" + diagnosticLatch.getSelection()
            + " left=[" + leftMotor.getDiagnosticStatus() + "]"
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
            diagnosticLatch.reset();
            stop();
        }
        SmartDashboard.putBoolean("Climber/Controllers Ready", isReady());
        SmartDashboard.putString("Climber/Diagnostic Selection", diagnosticLatch.getSelection());
        SmartDashboard.putString("Climber/Last Blocked Motion", lastBlockedMotion);
        SmartDashboard.putString("Climber/Status", getDiagnosticStatus());
    }
}
