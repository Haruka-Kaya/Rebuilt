package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class AlignmentCommand extends Command {

    private final CommandSwerveDrivetrain drive;
    private final double targetDistanceMeters;
    private final VisionSubsystem vision;
    private final CommandPS5Controller controller;

    private final boolean fieldRelative;

    // Gains
    private static final double kP_LINEAR = 1.2;    // m/s per meter
    private static final double kP_STRAFE = 1.2;    // m/s per meter
    private static final double kP_ROT = -0.05;     // Has to be negative

    // Tolerances
    private static final double DIST_TOL = 0.05;    // meters
    private static final double STRAFE_TOL = 0.05;  // meters
    private static final double ROT_TOL = Units.degreesToRadians(2.0);

    public AlignmentCommand(CommandSwerveDrivetrain drive, double targetDistanceMeters, VisionSubsystem vision, CommandPS5Controller controller, boolean fieldRelative) {
        this.fieldRelative = fieldRelative;
        this.drive = drive;
        this.targetDistanceMeters = targetDistanceMeters;
        this.vision = vision;
        this.controller = controller;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double rotation_speed = 0;

        if (vision.hasTarget()) {
            double tx = vision.getTx(); // degrees
            rotation_speed = -kP_ROT * tx;
        }

        drive.drive(
            -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband),
            rotation_speed,
            fieldRelative
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, fieldRelative);
    }

    
}
