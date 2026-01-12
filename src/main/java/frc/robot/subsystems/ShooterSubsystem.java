package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends ManipulatorSubsystem {
    private final SparkMax shooterMotor = new SparkMax(ShooterConstants.SHOOTER_CAN_ID, MotorType.kBrushless);
    
    public ShooterSubsystem() {

    }
}
