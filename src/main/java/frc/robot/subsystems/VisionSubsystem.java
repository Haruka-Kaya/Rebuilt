package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/** Provides frame-consistent, freshness-checked Limelight observations. */
public class VisionSubsystem extends SubsystemBase {
    static final double MAX_POSE_AGE_SECONDS = 0.30;
    static final double MAX_TARGET_AGE_SECONDS = 0.15;
    static final double MAX_TAG_DISTANCE_METERS = 8.0;
    static final double MAX_SINGLE_TAG_AMBIGUITY = 0.70;
    static final double MAX_HEARTBEAT_AGE_SECONDS = 0.50;

    private final NetworkTable limelight;
    private final String limelightName;
    private final int expectedPipeline;

    private double pipelineChangeTimestamp;
    private double lastSeenPoseTimestamp = -1.0;
    private double lastPoseTimestamp;
    private long lastTargetUpdateMicros = -1;
    private TargetObservation cachedTarget;

    public record PoseObservation(
            Pose2d pose,
            double timestampSeconds,
            int tagCount,
            double averageTagDistanceMeters,
            double maximumAmbiguity) {}

    public record TargetObservation(
            double timestampSeconds,
            double txDegrees,
            double tyDegrees,
            double areaPercent,
            int tagId) {}

    public VisionSubsystem(String limelightName) {
        this(limelightName, -1);
    }

    public VisionSubsystem(String limelightName, int expectedPipeline) {
        this.limelightName = limelightName;
        this.expectedPipeline = expectedPipeline;
        limelight = NetworkTableInstance.getDefault().getTable(limelightName);
        if (expectedPipeline >= 0) {
            setPipeline(expectedPipeline);
        }
    }

    /* ---------------- Pipeline Control ---------------- */

    public void setPipeline(int pipeline) {
        limelight.getEntry("pipeline").setNumber(pipeline);
        pipelineChangeTimestamp = Timer.getFPGATimestamp();
        lastSeenPoseTimestamp = -1.0;
        lastTargetUpdateMicros = -1;
        cachedTarget = null;
    }

    /** Returns the pipeline that the Limelight reports it is currently running. */
    public int getPipeline() {
        double reported = limelight.getEntry("getpipe").getDouble(-1.0);
        return Double.isFinite(reported)
                && reported >= 0.0
                && reported <= 9.0
                && reported == Math.rint(reported)
            ? (int) reported
            : -1;
    }

    private boolean isExpectedPipelineActive() {
        return expectedPipeline < 0 || getPipeline() == expectedPipeline;
    }

    /** Camera/pipeline preflight that does not require a target to already be visible. */
    public boolean isTargetingPipelineReady() {
        double heartbeatTimestampSeconds = limelight.getEntry("hb").getLastChange() / 1_000_000.0;
        return cameraPipelineReady(
            heartbeatTimestampSeconds,
            Timer.getFPGATimestamp(),
            getPipeline(),
            expectedPipeline);
    }

    static boolean cameraPipelineReady(
            double heartbeatTimestampSeconds,
            double nowSeconds,
            int reportedPipeline,
            int expectedPipeline) {
        if (!Double.isFinite(heartbeatTimestampSeconds)
                || heartbeatTimestampSeconds <= 0.0
                || !Double.isFinite(nowSeconds)) {
            return false;
        }
        double ageSeconds = nowSeconds - heartbeatTimestampSeconds;
        return ageSeconds >= 0.0
            && ageSeconds <= MAX_HEARTBEAT_AGE_SECONDS
            && reportedPipeline >= 0
            && (expectedPipeline < 0 || reportedPipeline == expectedPipeline);
    }

    /* ---------------- Target Validity ---------------- */

    /** Returns the newest complete target frame while it remains fresh. */
    public Optional<TargetObservation> getLatestTargetObservation() {
        var atomic = LimelightHelpers
                .getLimelightDoubleArrayEntry(limelightName, "t2d")
                .getAtomic();

        if (atomic.timestamp != lastTargetUpdateMicros) {
            lastTargetUpdateMicros = atomic.timestamp;
            cachedTarget = targetObservationFromFrame(
                    atomic.value,
                    atomic.timestamp / 1_000_000.0);
        }

        if (cachedTarget == null
                || !isExpectedPipelineActive()
                || cachedTarget.timestampSeconds() < pipelineChangeTimestamp
                || !isTimestampFresh(
                        cachedTarget.timestampSeconds(),
                        Timer.getFPGATimestamp(),
                        MAX_TARGET_AGE_SECONDS)) {
            return Optional.empty();
        }
        return Optional.of(cachedTarget);
    }

    static TargetObservation targetObservationFromFrame(
            double[] values,
            double networkTablesTimestampSeconds) {
        if (values == null
                || values.length != 17
                || values[0] != 1.0
                || !Double.isFinite(values[1])
                || values[1] < 1.0) {
            return null;
        }

        double targetLatencyMs = values[2];
        double captureLatencyMs = values[3];
        double latencyMs = targetLatencyMs + captureLatencyMs;
        double timestampSeconds = networkTablesTimestampSeconds - latencyMs / 1000.0;
        double tx = values[4];
        double ty = values[5];
        double area = values[8];
        double tagId = values[9];

        if (!Double.isFinite(networkTablesTimestampSeconds)
                || !Double.isFinite(targetLatencyMs)
                || !Double.isFinite(captureLatencyMs)
                || targetLatencyMs < 0.0
                || captureLatencyMs < 0.0
                || latencyMs > 500.0
                || !Double.isFinite(timestampSeconds)
                || !Double.isFinite(tx)
                || !Double.isFinite(ty)
                || !Double.isFinite(area)
                || !Double.isFinite(tagId)
                || tagId < 1.0
                || tagId > Integer.MAX_VALUE
                || tagId != Math.rint(tagId)) {
            return null;
        }

        return new TargetObservation(timestampSeconds, tx, ty, area, (int) tagId);
    }

    public boolean hasTarget() {
        return getLatestTargetObservation().isPresent();
    }

    /* ---------------- FUEL Targeting ---------------- */

    public double getTx() {
        return getLatestTargetObservation().map(TargetObservation::txDegrees).orElse(0.0);
    }

    public double getTy() {
        return getLatestTargetObservation().map(TargetObservation::tyDegrees).orElse(0.0);
    }

    public double getTa() {
        return getLatestTargetObservation().map(TargetObservation::areaPercent).orElse(0.0);
    }

    /* ---------------- AprilTag Pose ---------------- */

    /** Returns each valid Limelight pose frame at most once. */
    public Optional<PoseObservation> getEstimatedPose() {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (estimate == null
                || !Double.isFinite(estimate.timestampSeconds)
                || estimate.timestampSeconds <= lastSeenPoseTimestamp + 1e-6) {
            return Optional.empty();
        }

        double now = Timer.getFPGATimestamp();
        if (!isExpectedPipelineActive()
                || estimate.timestampSeconds < pipelineChangeTimestamp
                || !isPoseEstimateUsable(estimate, now)) {
            return Optional.empty();
        }

        // Invalid future/stale frames must not poison the monotonic frame guard.
        lastSeenPoseTimestamp = estimate.timestampSeconds;
        lastPoseTimestamp = estimate.timestampSeconds;
        return Optional.of(new PoseObservation(
                estimate.pose,
                estimate.timestampSeconds,
                estimate.tagCount,
                estimate.avgTagDist,
                maximumAmbiguity(estimate)));
    }

    static boolean isPoseEstimateUsable(PoseEstimate estimate, double nowSeconds) {
        if (estimate == null
                || estimate.pose == null
                || estimate.tagCount < 1
                || !Double.isFinite(estimate.timestampSeconds)
                || !Double.isFinite(estimate.latency)
                || estimate.latency < 0.0
                || estimate.latency > 500.0
                || !isTimestampFresh(
                        estimate.timestampSeconds,
                        nowSeconds,
                        MAX_POSE_AGE_SECONDS)) {
            return false;
        }

        double x = estimate.pose.getX();
        double y = estimate.pose.getY();
        double heading = estimate.pose.getRotation().getRadians();
        if (!Double.isFinite(x)
                || !Double.isFinite(y)
                || !Double.isFinite(heading)
                || x < -1.0
                || x > 20.0
                || y < -1.0
                || y > 10.0
                || !Double.isFinite(estimate.avgTagDist)
                || estimate.avgTagDist <= 0.0
                || estimate.avgTagDist > MAX_TAG_DISTANCE_METERS) {
            return false;
        }

        double ambiguity = maximumAmbiguity(estimate);
        return estimate.tagCount != 1
                || (Double.isFinite(ambiguity)
                    && ambiguity >= 0.0
                    && ambiguity <= MAX_SINGLE_TAG_AMBIGUITY);
    }

    private static double maximumAmbiguity(PoseEstimate estimate) {
        double maximum = Double.NaN;
        if (estimate.rawFiducials == null) {
            return maximum;
        }
        for (var fiducial : estimate.rawFiducials) {
            if (fiducial != null && Double.isFinite(fiducial.ambiguity)) {
                maximum = Double.isNaN(maximum)
                        ? fiducial.ambiguity
                        : Math.max(maximum, fiducial.ambiguity);
            }
        }
        return maximum;
    }

    private static boolean isTimestampFresh(
            double timestampSeconds,
            double nowSeconds,
            double maximumAgeSeconds) {
        double ageSeconds = nowSeconds - timestampSeconds;
        return Double.isFinite(ageSeconds)
                && ageSeconds >= -0.05
                && ageSeconds <= maximumAgeSeconds;
    }

    public double getLastPoseTimestamp() {
        return lastPoseTimestamp;
    }

    /* ---------------- Tag Info ---------------- */

    public int getPrimaryTagID() {
        return getLatestTargetObservation().map(TargetObservation::tagId).orElse(-1);
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
