package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.utils.TalonFxContainer;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFxContainer leftMotor = new TalonFxContainer(ClimberConstants.LEFT_MOTOR_CAN_ID);
    private final TalonFxContainer rightMotor = new TalonFxContainer(ClimberConstants.RIGHT_MOTOR_CAN_ID);

    public ClimberSubsystem() {
        
    }

    public void climb() {

    }

    public void extend() {

    }

    public void retract() {
        
    }

    @Override
    public void periodic() {
        
    }
}
