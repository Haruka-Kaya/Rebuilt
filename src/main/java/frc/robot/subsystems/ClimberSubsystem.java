package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.utils.TalonFxContainer;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFxContainer stinger = new TalonFxContainer(ClimberConstants.STINGER_CAN_ID);

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
