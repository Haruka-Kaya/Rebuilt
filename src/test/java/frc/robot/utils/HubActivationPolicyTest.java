package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

class HubActivationPolicyTest {
  @Test
  void followsEveryShiftWhenRedIsInactiveFirst() {
    assertTrue(active(Alliance.Red, 131.0, "R"));
    assertFalse(active(Alliance.Red, 120.0, "R"));
    assertTrue(active(Alliance.Red, 100.0, "R"));
    assertFalse(active(Alliance.Red, 75.0, "R"));
    assertTrue(active(Alliance.Red, 50.0, "R"));
    assertTrue(active(Alliance.Red, 20.0, "R"));

    assertTrue(active(Alliance.Blue, 120.0, "R"));
    assertFalse(active(Alliance.Blue, 100.0, "R"));
    assertTrue(active(Alliance.Blue, 75.0, "R"));
    assertFalse(active(Alliance.Blue, 50.0, "R"));
  }

  @Test
  void blueInactiveFirstMirrorsTheAlliances() {
    assertTrue(active(Alliance.Red, 120.0, "B"));
    assertFalse(active(Alliance.Blue, 120.0, "B"));
    assertFalse(active(Alliance.Red, 100.0, "B"));
    assertTrue(active(Alliance.Blue, 100.0, "B"));
  }

  @Test
  void exactBoundariesEnterTheFollowingPeriod() {
    assertFalse(active(Alliance.Red, 130.0, "R"));
    assertTrue(active(Alliance.Red, 105.0, "R"));
    assertFalse(active(Alliance.Red, 80.0, "R"));
    assertTrue(active(Alliance.Red, 55.0, "R"));
    assertTrue(active(Alliance.Red, 30.0, "R"));
  }

  @Test
  void modeAllianceAndMissingDataFollowTheOfficialPublishedBehavior() {
    assertFalse(HubActivationPolicy.isActive(null, false, true, 120.0, "R"));
    assertFalse(HubActivationPolicy.isActive(Alliance.Red, false, false, 120.0, "R"));
    assertTrue(HubActivationPolicy.isActive(Alliance.Red, true, false, 120.0, "R"));
    assertTrue(active(Alliance.Red, 120.0, ""));
    assertTrue(active(Alliance.Red, 120.0, "invalid"));
  }

  private static boolean active(Alliance alliance, double matchTimeSeconds, String data) {
    return HubActivationPolicy.isActive(
        alliance, false, true, matchTimeSeconds, data);
  }
}
