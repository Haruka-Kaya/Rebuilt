package frc.robot.utils;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Requires distinct Phoenix system-timestamp frames before a failed device may recover. */
public final class CtreSignalProgressTracker {
  private final int requiredAdvances;
  private final Map<String, Double> lastTimestamps = new LinkedHashMap<>();
  private int distinctAdvances;
  private boolean ready;

  public CtreSignalProgressTracker(int requiredAdvances) {
    if (requiredAdvances <= 0) {
      throw new IllegalArgumentException("required advances must be positive");
    }
    this.requiredAdvances = requiredAdvances;
  }

  public boolean observe(List<CtreDeviceEvidence.SignalObservation> observations) {
    if (!CtreDeviceEvidence.allFresh(observations)
        || observations.stream().anyMatch(
            observation -> !Double.isFinite(observation.progressTimestampSeconds()))) {
      reset();
      return false;
    }
    if (lastTimestamps.isEmpty()) {
      remember(observations);
      return false;
    }
    boolean rolledBack = observations.stream().anyMatch(observation ->
        observation.progressTimestampSeconds()
            < lastTimestamps.getOrDefault(observation.name(), Double.NEGATIVE_INFINITY));
    if (rolledBack) {
      reset();
      remember(observations);
      return false;
    }
    if (ready) {
      remember(observations);
      return true;
    }
    boolean allAdvanced = observations.stream().allMatch(observation ->
        observation.progressTimestampSeconds()
            > lastTimestamps.getOrDefault(observation.name(), Double.POSITIVE_INFINITY));
    if (allAdvanced) {
      distinctAdvances++;
    }
    remember(observations);
    ready = distinctAdvances >= requiredAdvances;
    return ready;
  }

  public int distinctAdvances() {
    return distinctAdvances;
  }

  public void reset() {
    lastTimestamps.clear();
    distinctAdvances = 0;
    ready = false;
  }

  private void remember(List<CtreDeviceEvidence.SignalObservation> observations) {
    for (var observation : observations) {
      lastTimestamps.put(observation.name(), observation.progressTimestampSeconds());
    }
  }
}
