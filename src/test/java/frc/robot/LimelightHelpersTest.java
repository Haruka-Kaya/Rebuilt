package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

class LimelightHelpersTest {
    @Test
    void parsePoseEstimateRejectsMalformedTagCountsBeforeAllocation() {
        assertNull(LimelightHelpers.parsePoseEstimate(poseArray(-1.0), 1_000_000L, false));
        assertNull(LimelightHelpers.parsePoseEstimate(
            poseArray(Double.NaN), 1_000_000L, false));
        assertNull(LimelightHelpers.parsePoseEstimate(
            poseArray(Integer.MAX_VALUE), 1_000_000L, false));
        assertNull(LimelightHelpers.parsePoseEstimate(poseArray(1.5), 1_000_000L, false));
    }

    @Test
    void parsePoseEstimateRequiresCompleteFiducialPayload() {
        assertNull(LimelightHelpers.parsePoseEstimate(poseArray(1.0), 1_000_000L, false));

        double[] complete = new double[18];
        complete[0] = 2.0;
        complete[1] = 3.0;
        complete[5] = 10.0;
        complete[6] = 20.0;
        complete[7] = 1.0;
        complete[9] = 2.0;
        complete[11] = 18.0;
        complete[17] = 0.15;

        var estimate = LimelightHelpers.parsePoseEstimate(
            complete, 1_000_000L, false);

        assertNotNull(estimate);
        assertEquals(0.98, estimate.timestampSeconds, 1e-9);
        assertEquals(1, estimate.tagCount);
        assertEquals(18, estimate.rawFiducials[0].id);
    }

    private static double[] poseArray(double tagCount) {
        double[] values = new double[11];
        values[7] = tagCount;
        return values;
    }
}
