package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.stream.IntStream;
import org.junit.jupiter.api.Test;

class HubTagFilterTest {
  private static final int[] RED_HUB_TAGS = {2, 3, 4, 5, 8, 9, 10, 11};
  private static final int[] BLUE_HUB_TAGS = {18, 19, 20, 21, 24, 25, 26, 27};
  private static final int[] NON_HUB_TAGS = {
      1, 6, 7, 12, 13, 14, 15, 16, 17, 22, 23, 28, 29, 30, 31, 32
  };

  @Test
  void acceptsEveryOfficialHubTagForItsAlliance() {
    IntStream.of(RED_HUB_TAGS)
        .forEach(id -> assertTrue(HubTagFilter.isHubTagForAlliance(Alliance.Red, id)));
    IntStream.of(BLUE_HUB_TAGS)
        .forEach(id -> assertTrue(HubTagFilter.isHubTagForAlliance(Alliance.Blue, id)));
  }

  @Test
  void rejectsOpposingAllianceAndNonHubTags() {
    IntStream.of(RED_HUB_TAGS)
        .forEach(id -> assertFalse(HubTagFilter.isHubTagForAlliance(Alliance.Blue, id)));
    IntStream.of(BLUE_HUB_TAGS)
        .forEach(id -> assertFalse(HubTagFilter.isHubTagForAlliance(Alliance.Red, id)));
    IntStream.of(NON_HUB_TAGS).forEach(id -> {
      assertFalse(HubTagFilter.isHubTagForAlliance(Alliance.Red, id));
      assertFalse(HubTagFilter.isHubTagForAlliance(Alliance.Blue, id));
    });
  }

  @Test
  void failsClosedForUnknownAllianceOrInvalidId() {
    assertFalse(HubTagFilter.isHubTagForAlliance(null, 2));
    assertFalse(HubTagFilter.isHubTagForAlliance(Alliance.Red, -1));
  }
}
