package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX leftStinger = new TalonFX(ClimberConstants.LEFT_STINGER_CAN_ID);
    private final TalonFX rightStinger = new TalonFX(ClimberConstants.RIGHT_STINGER_CAN_ID);

    public ClimberSubsystem() {
        
    }

    public void climbR1() {
        
    }

    @Override
    public void periodic() {
        
    }
}
