package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;

public class AlignmentCommand extends Command {
    private final double AprilTagHeight; // Height of AprilTag in inches

    private final DriveSubsystem swerve;
    private final PS5Controller cont;

    private final PIDController turnPID;
    private final PIDController forwardPID;
    private final PIDController strafePID;

    private static final double TARGET_ROT = 0.0; // tx
    private final double TARGET_FWD; // inches, distance to target
    private static final double TARGET_LR  = 0.0; // centered on target

    private final Timer stabilityTimer = new Timer();
    private static final double HOLD_TIME = 0.25;

    public AlignmentCommand(DriveSubsystem swerve, PS5Controller cont, double AprilTagHeightInches, double targetDistanceInches) {
        this.TARGET_FWD = targetDistanceInches;
        this.AprilTagHeight = AprilTagHeightInches;
        this.swerve = swerve;
        this.cont = cont;

        turnPID = new PIDController(0.1, 0, 0.00);
        forwardPID = new PIDController(0.1, 0.0, 0.0);
        strafePID = new PIDController(1.2, 0.0, 0.02);

        turnPID.setTolerance(1.0);
        forwardPID.setTolerance(0.05);
        strafePID.setTolerance(0.05);

        addRequirements(swerve);
    }

    @Override
    public void execute() {

        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

        NetworkTableEntry ty = limelight.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 0; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 17.0;

        // distance from the target to the floor (Adjust this)

        double angleToAprilTagDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToAprilTagRadians = angleToAprilTagDegrees * (Math.PI / 180.0);

        //calculate distance
        double distanceFromLimelightToAprilTagInches = (AprilTagHeight - limelightLensHeightInches) / Math.tan(angleToAprilTagRadians);


        double tx = limelight.getEntry("tx").getDouble(0.0);

        double[] targetPose = limelight.getEntry("targetpose_robotspcae").getDoubleArray(new double[6]);
        double y = targetPose[0];


        // double forwardSpeed = forwardPID.calculate(distanceFromLimelightToAprilTagInches, TARGET_FWD);

        double KpDistance = -0.1;
        double distance_error = TARGET_FWD - distanceFromLimelightToAprilTagInches;

        double forwardSpeed = KpDistance * distance_error;

        double strafeSpeed  = strafePID.calculate(y, TARGET_LR);
        
        double turnSpeed    = turnPID.calculate(tx, TARGET_ROT);

        // if (forwardPID.atSetpoint() || !limelight.getEntry("tv").getBoolean(false)) forwardSpeed = 0.0;
        
        if (strafePID.atSetpoint()) strafeSpeed = 0.0;
        if (turnPID.atSetpoint()) turnSpeed = 0.0;
        swerve.driveRobotRelative(new ChassisSpeeds(forwardSpeed, MathUtil.applyDeadband(cont.getLeftX(), OIConstants.kDriveDeadband), turnSpeed));

        if(forwardPID.atSetpoint() && strafePID.atSetpoint() && turnPID.atSetpoint()) {
            if(!stabilityTimer.isRunning()) {
                stabilityTimer.reset();
                stabilityTimer.start();
            }
        } else {
            stabilityTimer.stop();
            stabilityTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        stabilityTimer.stop();
        stabilityTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return stabilityTimer.hasElapsed(HOLD_TIME);
    }
}