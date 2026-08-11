package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.VisionSubsystem.TargetObservation;

class VisionSubsystemTest {
    @Test
    void targetObservationUsesOneAtomicFrameAndCompensatesLatency() {
        double[] frame = new double[17];
        frame[0] = 1.0;
        frame[1] = 2.0;
        frame[2] = 11.0;
        frame[3] = 4.0;
        frame[4] = -3.25;
        frame[5] = 1.5;
        frame[8] = 4.2;
        frame[9] = 18.0;

        TargetObservation observation =
            VisionSubsystem.targetObservationFromFrame(frame, 100.0);

        assertNotNull(observation);
        assertEquals(99.985, observation.timestampSeconds(), 1e-9);
        assertEquals(-3.25, observation.txDegrees(), 1e-9);
        assertEquals(1.5, observation.tyDegrees(), 1e-9);
        assertEquals(4.2, observation.areaPercent(), 1e-9);
        assertEquals(18, observation.tagId());
    }

    @Test
    void targetObservationRejectsInvalidOrIncompleteFrames() {
        assertNull(VisionSubsystem.targetObservationFromFrame(new double[6], 100.0));

        double[] noTarget = validTargetFrame();
        noTarget[0] = 0.0;
        assertNull(VisionSubsystem.targetObservationFromFrame(noTarget, 100.0));

        double[] nonFinite = validTargetFrame();
        nonFinite[4] = Double.NaN;
        assertNull(VisionSubsystem.targetObservationFromFrame(nonFinite, 100.0));

        double[] invalidCount = validTargetFrame();
        invalidCount[1] = Double.NaN;
        assertNull(VisionSubsystem.targetObservationFromFrame(invalidCount, 100.0));

        double[] invalidLatency = validTargetFrame();
        invalidLatency[2] = -1.0;
        assertNull(VisionSubsystem.targetObservationFromFrame(invalidLatency, 100.0));

        double[] fractionalTagId = validTargetFrame();
        fractionalTagId[9] = 18.9;
        assertNull(VisionSubsystem.targetObservationFromFrame(fractionalTagId, 100.0));
    }

    @Test
    void poseEstimateAcceptsFreshFiniteSingleTagObservation() {
        PoseEstimate estimate = poseEstimate(9.90, 1, 2.0, 0.15);

        assertTrue(VisionSubsystem.isPoseEstimateUsable(estimate, 10.0));
    }

    @Test
    void poseEstimateRejectsStaleFutureOrUnboundedObservations() {
        assertFalse(VisionSubsystem.isPoseEstimateUsable(
            poseEstimate(9.60, 1, 2.0, 0.15), 10.0));
        assertFalse(VisionSubsystem.isPoseEstimateUsable(
            poseEstimate(10.10, 1, 2.0, 0.15), 10.0));
        assertFalse(VisionSubsystem.isPoseEstimateUsable(
            poseEstimate(9.90, 0, 2.0, 0.15), 10.0));
        assertFalse(VisionSubsystem.isPoseEstimateUsable(
            poseEstimate(9.90, 1, 9.0, 0.15), 10.0));

        PoseEstimate outsideField = poseEstimate(9.90, 1, 2.0, 0.15);
        outsideField.pose = new Pose2d(25.0, 2.0, Rotation2d.kZero);
        assertFalse(VisionSubsystem.isPoseEstimateUsable(outsideField, 10.0));
    }

    @Test
    void poseEstimateRejectsAmbiguousSingleTagButAllowsMultiTag() {
        assertFalse(VisionSubsystem.isPoseEstimateUsable(
            poseEstimate(9.90, 1, 2.0, 0.80), 10.0));
        assertTrue(VisionSubsystem.isPoseEstimateUsable(
            poseEstimate(9.90, 2, 2.0, 0.80), 10.0));

        PoseEstimate missingAmbiguity = poseEstimate(9.90, 1, 2.0, 0.15);
        missingAmbiguity.rawFiducials = new RawFiducial[] { null };
        assertFalse(VisionSubsystem.isPoseEstimateUsable(missingAmbiguity, 10.0));

        assertFalse(VisionSubsystem.isPoseEstimateUsable(
            poseEstimate(9.90, 1, 2.0, -1.0), 10.0));
    }

    private static double[] validTargetFrame() {
        double[] frame = new double[17];
        frame[0] = 1.0;
        frame[1] = 1.0;
        frame[2] = 10.0;
        frame[3] = 5.0;
        frame[4] = 2.0;
        frame[5] = 1.0;
        frame[8] = 3.0;
        frame[9] = 18.0;
        return frame;
    }

    private static PoseEstimate poseEstimate(
            double timestampSeconds,
            int tagCount,
            double averageDistanceMeters,
            double ambiguity) {
        RawFiducial[] fiducials = new RawFiducial[Math.max(tagCount, 1)];
        for (int i = 0; i < fiducials.length; i++) {
            fiducials[i] = new RawFiducial(
                18 + i,
                0.0,
                0.0,
                1.0,
                averageDistanceMeters,
                averageDistanceMeters,
                ambiguity);
        }
        return new PoseEstimate(
            new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(10.0)),
            timestampSeconds,
            20.0,
            tagCount,
            0.5,
            averageDistanceMeters,
            1.0,
            fiducials,
            false);
    }
}
