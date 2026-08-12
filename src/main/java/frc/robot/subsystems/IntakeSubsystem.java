package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DiagnosticOutputSession.PulsePermit;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.HardwareTestConstants;
import frc.robot.utils.PositionReferenceGuard.Token;
import frc.robot.utils.DashboardApplyGate;
import frc.robot.utils.DashboardApplyGate.Decision;
import frc.robot.utils.SparkMAXContainer;
import frc.robot.utils.SparkMAXContainer.PositionCommandStatus;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.Snapshot;

public class IntakeSubsystem extends SubsystemBase {
    private static final String TUNING_APPLY_KEY = "Tuning/Intake/Apply";
    private static final String TUNING_STATUS_KEY = "Tuning/Intake/Status";
    private static final double MAX_PID_GAIN = 10.0;
    private static final double MAX_ACTUATOR_ANGLE_DEGREES = 10.0;
    private static final double MAX_ROLLER_DUTY_CYCLE = 0.20;

    private final SparkMAXContainer m_intakeRoller = new SparkMAXContainer(IntakeConstants.INTAKE_ROLLER_CAN_ID);
    private final SparkMAXContainer m_actuatorMotor = new SparkMAXContainer(IntakeConstants.INTAKE_ACTUATOR_CAN_ID);
    private final DashboardApplyGate tuningApplyGate = new DashboardApplyGate();
    // Assigned only after future, sensor-validated homing succeeds.
    private Token actuatorReference;
    private boolean actuatorDiagnosticActive;
    private String lastBlockedActuatorCommand = "startup: homing未実装";

    private double actuator_kP = 0.1;
    private double actuator_kI = 0.0;
    private double actuator_kD = 0.0;

    private double actuatorAngle = 10.0;
    
    private double slurpPercent = 0.15;
    private double spitPercent = -0.15;


    public IntakeSubsystem() {
        m_intakeRoller.setBreakMode(false);
        m_actuatorMotor.setBreakMode(true);
        m_intakeRoller.setCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT_AMPS);
        m_actuatorMotor.setCurrentLimit(15);

        m_actuatorMotor.assignPIDValues(actuator_kP, actuator_kI, actuator_kD);

        SmartDashboard.putNumber("Set intake actuator_kP", 0.1);
        SmartDashboard.putNumber("Set intake actuator_kI", 0);
        SmartDashboard.putNumber("Set intake actuator_kD", 0);
        
        SmartDashboard.putNumber("Set intake actuator degrees", 10);

        SmartDashboard.putNumber("Set slurp roller percent", 0.15);
        SmartDashboard.putNumber("Set spit roller percent", -0.15);
        SmartDashboard.putBoolean(TUNING_APPLY_KEY, false);
        SmartDashboard.putString(TUNING_STATUS_KEY, "ACTIVE_DEFAULTS");
    }

    private boolean slurp() {
        return m_intakeRoller.setDutyCycle(slurpPercent);
    }

    private boolean spit() {
        return m_intakeRoller.setDutyCycle(spitPercent);
    }

    public boolean runIntake(boolean trueForIn) {
        return trueForIn ? slurp() : spit();
    }

    /** Returns true only when a referenced actuator has reached the requested extension. */
    public PositionCommandStatus extendIntake() {
        return commandActuatorDegrees(actuatorAngle, "extend");
    }

    public PositionCommandStatus retractIntake() {
        return commandActuatorDegrees(0.0, "retract");
    }

    private PositionCommandStatus commandActuatorDegrees(double degrees, String commandName) {
        if (!m_actuatorMotor.isPositionReferenceValid(actuatorReference)) {
            lastBlockedActuatorCommand = commandName + ": UNREFERENCED";
            m_actuatorMotor.stop();
            return PositionCommandStatus.REJECTED;
        }
        PositionCommandStatus status = m_actuatorMotor.commandReferencedPosition(
            degrees / 360.0, 0.5 / 360.0, actuatorReference);
        if (status == PositionCommandStatus.REJECTED) {
            lastBlockedActuatorCommand = commandName + ": command rejected";
        }
        return status;
    }

    public boolean isActuatorReferenced() {
        return m_actuatorMotor.isPositionReferenceValid(actuatorReference);
    }

    private OptionalDouble getReferencedActuatorDegrees() {
        OptionalDouble rotations = m_actuatorMotor.getReferencedPosition(actuatorReference);
        return rotations.isPresent()
            ? OptionalDouble.of(rotations.getAsDouble() * 360.0)
            : OptionalDouble.empty();
    }

    public void stop() {
        stopAll();
    }

    public void stopRoller() {
        m_intakeRoller.stop();
    }

    public boolean runRollerDiagnostic(double requestedDuty, PulsePermit permit) {
        if (!DriverStation.isTestEnabled()
                || DriverStation.isFMSAttached()
                || !Double.isFinite(requestedDuty)
                || Math.abs(requestedDuty) > HardwareTestConstants.OPEN_LOOP_DUTY_CYCLE
                || permit == null
                || !permit.isValidFor(requestedDuty)) {
            stopRoller();
            return false;
        }
        return m_intakeRoller.setDutyCycleIfAuthorized(
            requestedDuty,
            () -> permit.isValidFor(requestedDuty)
                && DriverStation.isTestEnabled()
                && !DriverStation.isFMSAttached());
    }

    public Snapshot getRollerDiagnosticSnapshot(boolean commandAccepted) {
        return m_intakeRoller.getDiagnosticSnapshot(commandAccepted);
    }

    /** Test-only low-output polarity evidence; this does not establish a position reference. */
    public boolean runUnhomedActuatorDiagnostic(
            double requestedDuty, PulsePermit permit) {
        if (!unhomedDiagnosticAllowed(requestedDuty, permit)) {
            stopActuatorDiagnostic();
            return false;
        }
        actuatorDiagnosticActive = true;
        boolean accepted = m_actuatorMotor.setDutyCycleIfAuthorized(
            requestedDuty,
            () -> permit.isValidFor(requestedDuty)
                && DriverStation.isTestEnabled()
                && !DriverStation.isFMSAttached());
        if (!accepted) {
            stopActuatorDiagnostic();
        }
        return accepted;
    }

    public Snapshot getActuatorDiagnosticSnapshot(boolean commandAccepted) {
        return m_actuatorMotor.getDiagnosticSnapshot(commandAccepted);
    }

    public void stopActuatorDiagnostic() {
        actuatorDiagnosticActive = false;
        m_actuatorMotor.stop();
    }

    public void stopAll() {
        actuatorDiagnosticActive = false;
        m_intakeRoller.stop();
        m_actuatorMotor.stop();
    }
    @Override
    public void periodic() {
        if (!isActuatorReferenced() && !actuatorDiagnosticActive) {
            m_actuatorMotor.stop();
        }
        OptionalDouble positionDegrees = getReferencedActuatorDegrees();
        OptionalDouble rawRotations = m_actuatorMotor.getPositionIfReady();
        SmartDashboard.putBoolean("Intake Actuator/Controller Ready", m_actuatorMotor.isReady());
        SmartDashboard.putString(
            "Intake Actuator/Reference State",
            m_actuatorMotor.getPositionReferenceStatus(actuatorReference));
        SmartDashboard.putNumber(
            "Intake Actuator/Continuity Epoch", m_actuatorMotor.getPositionContinuityEpoch());
        SmartDashboard.putBoolean("Intake Actuator/Position Valid", positionDegrees.isPresent());
        SmartDashboard.putNumber(
            "Intake Actuator/Raw Encoder Rotations", rawRotations.orElse(Double.NaN));
        SmartDashboard.putNumber(
            "Real intake actuator degrees", positionDegrees.orElse(Double.NaN));
        SmartDashboard.putString("Intake Actuator/Last Blocked Command", lastBlockedActuatorCommand);

        processDashboardTuning();
    }

    private static boolean unhomedDiagnosticAllowed(
            double requestedDuty, PulsePermit permit) {
        return DriverStation.isTestEnabled()
            && !DriverStation.isFMSAttached()
            && Double.isFinite(requestedDuty)
            && Math.abs(requestedDuty)
                <= HardwareTestConstants.UNHOMED_DIAGNOSTIC_MAX_DUTY_CYCLE
            && permit != null
            && permit.isValidFor(requestedDuty);
    }

    private void processDashboardTuning() {
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

        double requestedP = SmartDashboard.getNumber("Set intake actuator_kP", actuator_kP);
        double requestedI = SmartDashboard.getNumber("Set intake actuator_kI", actuator_kI);
        double requestedD = SmartDashboard.getNumber("Set intake actuator_kD", actuator_kD);
        double requestedAngle = SmartDashboard.getNumber(
            "Set intake actuator degrees", actuatorAngle);
        double requestedSlurp = SmartDashboard.getNumber(
            "Set slurp roller percent", slurpPercent);
        double requestedSpit = SmartDashboard.getNumber(
            "Set spit roller percent", spitPercent);

        boolean valid = DashboardApplyGate.allFiniteInRange(
            new double[] {
                requestedP, requestedI, requestedD,
                requestedAngle, requestedSlurp, requestedSpit
            },
            new double[] {0.0, 0.0, 0.0, 0.0, 0.0, -MAX_ROLLER_DUTY_CYCLE},
            new double[] {
                MAX_PID_GAIN, MAX_PID_GAIN, MAX_PID_GAIN,
                MAX_ACTUATOR_ANGLE_DEGREES, MAX_ROLLER_DUTY_CYCLE, 0.0
            });
        if (!valid) {
            publishActiveTuning();
            SmartDashboard.putString(TUNING_STATUS_KEY, "REJECTED_INVALID_OR_WRONG_SIGN");
            return;
        }

        if (pidChanged(requestedP, requestedI, requestedD)) {
            actuator_kP = requestedP;
            actuator_kI = requestedI;
            actuator_kD = requestedD;
            m_actuatorMotor.assignPIDValues(actuator_kP, actuator_kI, actuator_kD);
        }
        actuatorAngle = requestedAngle;
        slurpPercent = requestedSlurp;
        spitPercent = requestedSpit;
        SmartDashboard.putString(TUNING_STATUS_KEY, "QUEUED_DISABLED");
    }

    private boolean pidChanged(double requestedP, double requestedI, double requestedD) {
        return Double.compare(actuator_kP, requestedP) != 0
            || Double.compare(actuator_kI, requestedI) != 0
            || Double.compare(actuator_kD, requestedD) != 0;
    }

    private void publishActiveTuning() {
        SmartDashboard.putNumber("Set intake actuator_kP", actuator_kP);
        SmartDashboard.putNumber("Set intake actuator_kI", actuator_kI);
        SmartDashboard.putNumber("Set intake actuator_kD", actuator_kD);
        SmartDashboard.putNumber("Set intake actuator degrees", actuatorAngle);
        SmartDashboard.putNumber("Set slurp roller percent", slurpPercent);
        SmartDashboard.putNumber("Set spit roller percent", spitPercent);
    }
}
