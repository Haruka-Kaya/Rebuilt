package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.utils.PositionReferenceGuard.Token;
import frc.robot.utils.SparkMAXContainer;
import frc.robot.utils.SparkMAXContainer.PositionCommandStatus;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMAXContainer m_intakeRoller = new SparkMAXContainer(IntakeConstants.INTAKE_ROLLER_CAN_ID);
    private final SparkMAXContainer m_actuatorMotor = new SparkMAXContainer(IntakeConstants.INTAKE_ACTUATOR_CAN_ID);
    // Assigned only after future, sensor-validated homing succeeds.
    private Token actuatorReference;
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
        m_intakeRoller.setCurrentLimit(20);
        m_actuatorMotor.setCurrentLimit(15);

        m_actuatorMotor.assignPIDValues(actuator_kP, actuator_kI, actuator_kD);

        SmartDashboard.putNumber("Set intake actuator_kP", 0.1);
        SmartDashboard.putNumber("Set intake actuator_kI", 0);
        SmartDashboard.putNumber("Set intake actuator_kD", 0);
        
        SmartDashboard.putNumber("Set intake actuator degrees", 10);

        SmartDashboard.putNumber("Set slurp roller percent", 0.15);
        SmartDashboard.putNumber("Set spit roller percent", -0.15);
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

    public void stopAll() {
        m_intakeRoller.stop();
        m_actuatorMotor.stop();
    }
    @Override
    public void periodic() {
        if (!isActuatorReferenced()) {
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

        double requestedActuatorKp = SmartDashboard.getNumber("Set intake actuator_kP", 0.1);
        double requestedActuatorKi = SmartDashboard.getNumber("Set intake actuator_kI", 0);
        double requestedActuatorKd = SmartDashboard.getNumber("Set intake actuator_kD", 0);

        if (Double.compare(actuator_kP, requestedActuatorKp) != 0
                || Double.compare(actuator_kI, requestedActuatorKi) != 0
                || Double.compare(actuator_kD, requestedActuatorKd) != 0) {
            actuator_kP = requestedActuatorKp;
            actuator_kI = requestedActuatorKi;
            actuator_kD = requestedActuatorKd;
            m_actuatorMotor.assignPIDValues(actuator_kP, actuator_kI, actuator_kD);
        }
        actuatorAngle = MathUtil.clamp(
            SmartDashboard.getNumber("Set intake actuator degrees", 10), 0, 10);

        slurpPercent = MathUtil.clamp(
            SmartDashboard.getNumber("Set slurp roller percent", 0.15), -0.20, 0.20);
        spitPercent = MathUtil.clamp(
            SmartDashboard.getNumber("Set spit roller percent", -0.15), -0.20, 0.20);
    }
}
