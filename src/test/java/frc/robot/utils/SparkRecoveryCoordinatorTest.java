package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

class SparkRecoveryCoordinatorTest {
  @Test
  void enforcesSingleWorkerDisabledGateAndGlobalCadence() {
    SparkRecoveryCoordinator coordinator = new SparkRecoveryCoordinator();

    assertTrue(coordinator.selectConfiguration(0.0, 3, false, true, i -> true).isEmpty());
    assertTrue(coordinator.selectConfiguration(0.0, 3, true, false, i -> true).isEmpty());
    assertEquals(0, coordinator.selectConfiguration(0.0, 3, true, true, i -> true).orElseThrow());
    assertTrue(coordinator.selectConfiguration(0.19, 3, true, true, i -> true).isEmpty());
    assertEquals(1, coordinator.selectConfiguration(0.20, 3, true, true, i -> true).orElseThrow());
  }

  @Test
  void visitsDueDevicesRoundRobinWithoutStarvation() {
    SparkRecoveryCoordinator coordinator = new SparkRecoveryCoordinator();
    List<Integer> selected = new ArrayList<>();

    for (int step = 0; step < 6; step++) {
      selected.add(coordinator
          .selectConfiguration(step * 0.20, 3, true, true, i -> i != 1)
          .orElseThrow());
    }

    assertEquals(List.of(0, 2, 0, 2, 0, 2), selected);
  }
}
