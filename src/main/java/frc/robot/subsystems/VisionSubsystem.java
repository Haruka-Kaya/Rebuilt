package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable limelight;

    // Cache latency handling
    private double lastPoseTimestamp = 0;

    public VisionSubsystem(String limelightName) {
        limelight = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    /* ---------------- Pipeline Control ---------------- */

    public void setPipeline(int pipeline) {
        limelight.getEntry("pipeline").setNumber(pipeline);
    }

    public int getPipeline() {
        return limelight.getEntry("pipeline").getNumber(0).intValue();
    }

    /* ---------------- Target Validity ---------------- */

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1.0;
    }

    /* ---------------- FUEL Targeting ---------------- */

    public double getTx() {
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public double getTa() {
        return limelight.getEntry("ta").getDouble(0.0);
    }

    /* ---------------- AprilTag Pose ---------------- */

    /**
     * Returns robot pose estimated by Limelight (WPILib field coords).
     * Automatically handles latency compensation.
     */
    public Optional<Pose2d> getEstimatedPose() {
        double[] botpose = limelight
                .getEntry("botpose_wpiblue")
                .getDoubleArray(new double[6]);

        if (botpose.length < 6 || !hasTarget()) {
            return Optional.empty();
        }

        Pose2d pose = new Pose2d(
                botpose[0],
                botpose[1],
                Rotation2d.fromDegrees(botpose[5])
        );

        double latencyMs =
                limelight.getEntry("tl").getDouble(0) +
                limelight.getEntry("cl").getDouble(0);

        lastPoseTimestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);

        return Optional.of(pose);
    }

    public double getLastPoseTimestamp() {
        return lastPoseTimestamp;
    }

    /* ---------------- Tag Info ---------------- */

    public int getPrimaryTagID() {
        return (int) limelight.getEntry("tid").getDouble(-1);
    }

    public boolean isTrackingTag(int tagID) {
        return getPrimaryTagID() == tagID;
    }

    /* ---------------- Utilities ---------------- */

    public void setLED(boolean on) {
        limelight.getEntry("ledMode").setNumber(on ? 3 : 1);
    }

    public void setCameraModeVision() {
        limelight.getEntry("camMode").setNumber(0);
    }

    public void setCameraModeDriver() {
        limelight.getEntry("camMode").setNumber(1);
    }
}
