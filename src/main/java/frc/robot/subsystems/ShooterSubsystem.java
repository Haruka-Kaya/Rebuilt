package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.HardwareTestConstants;
import frc.robot.utils.DashboardApplyGate;
import frc.robot.utils.DashboardApplyGate.Decision;
import frc.robot.utils.PositionReferenceGuard.Token;
import frc.robot.utils.SparkMAXContainer;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.Snapshot;

public class ShooterSubsystem extends SubsystemBase {
    private static final String TUNING_APPLY_KEY = "Tuning/Shooter/Apply";
    private static final String TUNING_STATUS_KEY = "Tuning/Shooter/Status";
    private static final double MAX_PID_GAIN = 10.0;
    private static final double MIN_FLYWHEEL_RPM = 100.0;
    private static final double MAX_FLYWHEEL_RPM = 1000.0;
    private static final double FLYWHEEL_TOLERANCE_RPM = 50.0;
    private static final double MAX_ACTUATOR_ANGLE_DEGREES = 5.0;

    private final SparkMAXContainer actuatorMotor = new SparkMAXContainer(ShooterConstants.ACTUATOR_CAN_ID);
    private final SparkMAXContainer flywheelMotor_1 = new SparkMAXContainer(ShooterConstants.SHOOTER_1_CAN_ID);
    private final SparkMAXContainer flywheelMotor_2 = new SparkMAXContainer(ShooterConstants.SHOOTER_2_CAN_ID);
    private final DashboardApplyGate tuningApplyGate = new DashboardApplyGate();
    // Assigned only after future, sensor-validated homing succeeds.
    private Token actuatorReference;
    private String lastBlockedActuatorCommand = "startup: homing未実装";

    private double flywheelRPM = 500.0;

    private double flywheelkP = 0.1;
    private double flywheelkI = 0.0;
    private double flywheelkD = 0.0;

    private boolean flywheelIsSet = false;
    private boolean flywheelRequested;

    
    private double actuatorPos = 5.0;

    private double actuatorkP = 0.1;
    private double actuatorkI = 0.0;
    private double actuatorkD = 0.0;

    public ShooterSubsystem() {
        flywheelMotor_1.assignPIDValues(flywheelkP, flywheelkI, flywheelkD);
        flywheelMotor_2.setupAsFollowerMotor(flywheelMotor_1, true);
        actuatorMotor.assignPIDValues(actuatorkP, actuatorkI, actuatorkD);

        actuatorMotor.setBreakMode(true);
        flywheelMotor_1.setBreakMode(false);
        flywheelMotor_2.setBreakMode(false);
        actuatorMotor.setCurrentLimit(15);
        flywheelMotor_1.setCurrentLimit(ShooterConstants.FLYWHEEL_CURRENT_LIMIT_AMPS);
        flywheelMotor_2.setCurrentLimit(ShooterConstants.FLYWHEEL_CURRENT_LIMIT_AMPS);

        SmartDashboard.putNumber("Set flywheel_kP", 0.1);
        SmartDashboard.putNumber("Set flywheel_kI", 0);
        SmartDashboard.putNumber("Set flywheel_kD", 0);

        SmartDashboard.putNumber("Set flywheelRPM", 500);


        SmartDashboard.putNumber("Set shooter actuator_kP", 0.1);
        SmartDashboard.putNumber("Set shooter actuator_kI", 0);
        SmartDashboard.putNumber("Set shooter actuator_kD", 0);

        SmartDashboard.putNumber("Set shooter actuator degrees", 5);
        SmartDashboard.putBoolean(TUNING_APPLY_KEY, false);
        SmartDashboard.putString(TUNING_STATUS_KEY, "ACTIVE_DEFAULTS");
    }

    /**
     * 
     * Set shooter speed based off network table values
     * 
     */
    public void setShooterSpeed() {
        if (!flywheelPairReady()) {
            flywheelMotor_1.stop();
            flywheelRequested = false;
            flywheelIsSet = false;
            return;
        }
        flywheelRequested = true;
        if (!flywheelMotor_1.setVelocity(flywheelRPM)) {
            flywheelRequested = false;
            flywheelIsSet = false;
            flywheelMotor_1.stop();
        }
    }

    /** Commands both shot mechanisms; the existing feed interlock still requires both at target. */
    public void prepareToFire() {
        setActuatorAngle();
        setShooterSpeed();
    }

    public boolean setActuatorAngle() {
        return setActuatorAngle(actuatorPos);
    }

    public boolean setActuatorAngle(double degrees) {
        if (!Double.isFinite(degrees)) {
            lastBlockedActuatorCommand = "set angle: invalid value";
            actuatorMotor.stop();
            return false;
        }
        double safeDegrees = MathUtil.clamp(degrees, 0.0, 5.0);
        if (!actuatorMotor.isPositionReferenceValid(actuatorReference)) {
            lastBlockedActuatorCommand = "set angle: UNREFERENCED";
            actuatorMotor.stop();
            return false;
        }
        boolean atTarget = actuatorMotor.goToReferencedPosition(
            safeDegrees / 360.0, 0.5 / 360.0, actuatorReference);
        if (!atTarget) {
            lastBlockedActuatorCommand = "set angle: not at target or command rejected";
        }
        return atTarget;
    }

    public boolean isActuatorReferenced() {
        return actuatorMotor.isPositionReferenceValid(actuatorReference);
    }

    private OptionalDouble getReferencedActuatorDegrees() {
        OptionalDouble rotations = actuatorMotor.getReferencedPosition(actuatorReference);
        return rotations.isPresent()
            ? OptionalDouble.of(rotations.getAsDouble() * 360.0)
            : OptionalDouble.empty();
    }

    public void stop() {
        flywheelRequested = false;
        flywheelIsSet = false;
        actuatorMotor.stop();
        flywheelMotor_1.stop();
        flywheelMotor_2.stop();
    }

    @Override
    public void periodic() {
        if (!isActuatorReferenced()) {
            actuatorMotor.stop();
        }
        processDashboardTuning();

        OptionalDouble actuatorDegrees = getReferencedActuatorDegrees();
        OptionalDouble rawActuatorRotations = actuatorMotor.getPositionIfReady();
        SmartDashboard.putBoolean("Shooter Actuator/Controller Ready", actuatorMotor.isReady());
        SmartDashboard.putString(
            "Shooter Actuator/Reference State",
            actuatorMotor.getPositionReferenceStatus(actuatorReference));
        SmartDashboard.putNumber(
            "Shooter Actuator/Continuity Epoch", actuatorMotor.getPositionContinuityEpoch());
        SmartDashboard.putBoolean("Shooter Actuator/Position Valid", actuatorDegrees.isPresent());
        SmartDashboard.putNumber(
            "Shooter Actuator/Raw Encoder Rotations",
            rawActuatorRotations.orElse(Double.NaN));
        SmartDashboard.putNumber(
            "Real shooter acutator degrees", actuatorDegrees.orElse(Double.NaN));
        SmartDashboard.putString(
            "Shooter Actuator/Last Blocked Command", lastBlockedActuatorCommand);
        SmartDashboard.putNumber("Real flywheelRPM", flywheelMotor_1.getVelocity());

        boolean diagnosticActive = flywheelMotor_2.isFollowerDiagnosticActive();
        boolean pairReady = flywheelPairReady();
        if (!diagnosticActive && !pairReady) {
            flywheelMotor_1.stop();
        }
        flywheelIsSet = isFlywheelReady();

        SmartDashboard.putBoolean("Flywheel reved up", flywheelIsSet);
        SmartDashboard.putBoolean("Shooter Ready To Feed", isReadyToFeed());
    }

    public boolean runFollowerDiagnostic() {
        flywheelRequested = false;
        flywheelIsSet = false;
        flywheelMotor_1.stop();
        return flywheelMotor_2.beginFollowerDiagnostic(0.08);
    }

    public boolean runFlywheelPairDiagnostic(double requestedDuty) {
        flywheelRequested = false;
        flywheelIsSet = false;
        if (!DriverStation.isTestEnabled()
                || DriverStation.isFMSAttached()
                || !Double.isFinite(requestedDuty)
                || Math.abs(requestedDuty) > HardwareTestConstants.OPEN_LOOP_DUTY_CYCLE
                || !flywheelPairReady()) {
            flywheelMotor_1.stop();
            return false;
        }
        return flywheelMotor_1.setDutyCycle(requestedDuty);
    }

    public void stopFlywheelPairDiagnostic() {
        flywheelMotor_1.stop();
        flywheelMotor_2.stop();
    }

    public Snapshot getFlywheelLeaderDiagnosticSnapshot(boolean commandAccepted) {
        return flywheelMotor_1.getDiagnosticSnapshot(commandAccepted);
    }

    public Snapshot getFlywheelFollowerDiagnosticSnapshot(boolean commandAccepted) {
        return flywheelMotor_2.getDiagnosticSnapshot(commandAccepted);
    }

    public Snapshot getIsolatedFollowerDiagnosticSnapshot(boolean commandAccepted) {
        return flywheelMotor_2.getFollowerDiagnosticSnapshot(commandAccepted);
    }

    public boolean isFollowerDiagnosticActive() {
        return flywheelMotor_2.isFollowerDiagnosticActive();
    }

    public boolean isFollowerDiagnosticOutputSafe() {
        return flywheelMotor_2.isFollowerDiagnosticOutputSafe();
    }

    public boolean isFollowerDiagnosticTransitionSafe() {
        return flywheelMotor_2.isFollowerDiagnosticTransitionSafe();
    }

    public void stopFollowerDiagnostic() {
        flywheelMotor_2.endFollowerDiagnostic();
    }

    public String getFollowerDiagnosticStatus() {
        return flywheelMotor_2.getDiagnosticStatus();
    }

    public boolean isFlywheelReady() {
        return flywheelRequested
            && Math.abs(flywheelRPM) > FLYWHEEL_TOLERANCE_RPM
            && !flywheelMotor_2.isFollowerDiagnosticActive()
            && flywheelPairReady()
            && flywheelAtRequestedSpeed(flywheelMotor_1.getVelocity())
            && flywheelAtRequestedSpeed(flywheelMotor_2.getVelocity());
    }

    /** Feed interlock: an unknown hood/actuator angle must never release a game piece. */
    public boolean isReadyToFeed() {
        OptionalDouble actuatorDegrees = getReferencedActuatorDegrees();
        return isFlywheelReady()
            && actuatorDegrees.isPresent()
            && MathUtil.isNear(actuatorPos, actuatorDegrees.getAsDouble(), 0.5);
    }

    private boolean flywheelPairReady() {
        return flywheelMotor_1.isReady() && flywheelMotor_2.isReady();
    }

    private boolean flywheelAtRequestedSpeed(double measuredVelocity) {
        return MathUtil.isNear(
            Math.abs(flywheelRPM), Math.abs(measuredVelocity), FLYWHEEL_TOLERANCE_RPM);
    }

    private static boolean pidChanged(
            double currentP, double currentI, double currentD,
            double requestedP, double requestedI, double requestedD) {
        return Double.compare(currentP, requestedP) != 0
                || Double.compare(currentI, requestedI) != 0
                || Double.compare(currentD, requestedD) != 0;
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

        double requestedFlywheelP = SmartDashboard.getNumber("Set flywheel_kP", flywheelkP);
        double requestedFlywheelI = SmartDashboard.getNumber("Set flywheel_kI", flywheelkI);
        double requestedFlywheelD = SmartDashboard.getNumber("Set flywheel_kD", flywheelkD);
        double requestedRpm = SmartDashboard.getNumber("Set flywheelRPM", flywheelRPM);
        double requestedActuatorP = SmartDashboard.getNumber(
            "Set shooter actuator_kP", actuatorkP);
        double requestedActuatorI = SmartDashboard.getNumber(
            "Set shooter actuator_kI", actuatorkI);
        double requestedActuatorD = SmartDashboard.getNumber(
            "Set shooter actuator_kD", actuatorkD);
        double requestedActuatorPosition = SmartDashboard.getNumber(
            "Set shooter actuator degrees", actuatorPos);

        boolean valid = DashboardApplyGate.allFiniteInRange(
            new double[] {
                requestedFlywheelP, requestedFlywheelI, requestedFlywheelD, requestedRpm,
                requestedActuatorP, requestedActuatorI, requestedActuatorD,
                requestedActuatorPosition
            },
            new double[] {0.0, 0.0, 0.0, MIN_FLYWHEEL_RPM, 0.0, 0.0, 0.0, 0.0},
            new double[] {
                MAX_PID_GAIN, MAX_PID_GAIN, MAX_PID_GAIN, MAX_FLYWHEEL_RPM,
                MAX_PID_GAIN, MAX_PID_GAIN, MAX_PID_GAIN, MAX_ACTUATOR_ANGLE_DEGREES
            });
        if (!valid) {
            publishActiveTuning();
            SmartDashboard.putString(TUNING_STATUS_KEY, "REJECTED_INVALID_OR_OUT_OF_RANGE");
            return;
        }

        if (pidChanged(
                flywheelkP, flywheelkI, flywheelkD,
                requestedFlywheelP, requestedFlywheelI, requestedFlywheelD)) {
            flywheelkP = requestedFlywheelP;
            flywheelkI = requestedFlywheelI;
            flywheelkD = requestedFlywheelD;
            flywheelMotor_1.assignPIDValues(flywheelkP, flywheelkI, flywheelkD);
        }
        if (pidChanged(
                actuatorkP, actuatorkI, actuatorkD,
                requestedActuatorP, requestedActuatorI, requestedActuatorD)) {
            actuatorkP = requestedActuatorP;
            actuatorkI = requestedActuatorI;
            actuatorkD = requestedActuatorD;
            actuatorMotor.assignPIDValues(actuatorkP, actuatorkI, actuatorkD);
        }
        flywheelRPM = requestedRpm;
        actuatorPos = requestedActuatorPosition;
        SmartDashboard.putString(TUNING_STATUS_KEY, "QUEUED_DISABLED");
    }

    private void publishActiveTuning() {
        SmartDashboard.putNumber("Set flywheel_kP", flywheelkP);
        SmartDashboard.putNumber("Set flywheel_kI", flywheelkI);
        SmartDashboard.putNumber("Set flywheel_kD", flywheelkD);
        SmartDashboard.putNumber("Set flywheelRPM", flywheelRPM);
        SmartDashboard.putNumber("Set shooter actuator_kP", actuatorkP);
        SmartDashboard.putNumber("Set shooter actuator_kI", actuatorkI);
        SmartDashboard.putNumber("Set shooter actuator_kD", actuatorkD);
        SmartDashboard.putNumber("Set shooter actuator degrees", actuatorPos);
    }
}
