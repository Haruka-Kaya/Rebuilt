package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AlignmentCommand extends Command {

    private final DriveSubsystem m_swerve;

    private final double TARGET_HEIGHT_METERS;

    private final Timer stabilityTimer = new Timer();
    private static final double HOLD_TIME = 0.25;

    public AlignmentCommand(DriveSubsystem swerve, double target_height_meters) {
        this.m_swerve = swerve;
        this.TARGET_HEIGHT_METERS = target_height_meters;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV("")) {
            m_swerve.stop();
            return;
        }
        double angleErrRad = -Units.degreesToRadians(LimelightHelpers.getTX(""));
        double forwardDistErrMeters = getDistanceToTargetMeters(); 
        double strafeDistErrMeters = forwardDistErrMeters * Math.tan(angleErrRad);

        if (LimelightHelpers.getTV("")) {
        m_swerve.driveRobotRelative(forwardDistErrMeters, strafeDistErrMeters, angleErrRad * 3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        stabilityTimer.stop();
        stabilityTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return stabilityTimer.hasElapsed(HOLD_TIME);
    }

    /*
     * 
     * Utils
     * 
     */

    public double getDistanceToTargetMeters() {
        Rotation2d angleToGoal = Rotation2d.fromDegrees(LimelightConstants.MOUNT_ANGLE_DEG)
            .plus(Rotation2d.fromDegrees(LimelightHelpers.getTY("")));

        double distance = (LimelightConstants.HEIGHT_METERS - TARGET_HEIGHT_METERS)
            / Math.tan(Math.abs(angleToGoal.getRadians()));
            
        return distance;
    }

}