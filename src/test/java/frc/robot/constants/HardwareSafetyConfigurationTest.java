package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.constants.Constants.ManipulatorConstants;
import org.junit.jupiter.api.Test;

class HardwareSafetyConfigurationTest {
  @Test
  void feederKnownStallRemainsBlockedUntilPhysicalRetest() {
    assertTrue(ManipulatorConstants.FEEDER_MOTION_BLOCKED_KNOWN_STALL);
  }
}
