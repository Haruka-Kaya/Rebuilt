package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ConfiguredCanHardware;
import frc.robot.constants.ConfiguredCanHardware.Device;
import frc.robot.constants.ConfiguredCanHardware.Vendor;
import frc.robot.utils.SparkMAXContainer.DeviceEvidenceSnapshot;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/** Manifest-driven, change-only SmartDashboard evidence for configured REV SPARK devices. */
public final class SparkDeviceEvidence {
  public static final String DASHBOARD_PREFIX = "Hardware/Device/";
  public static final String NETWORK_TABLES_PREFIX = "/SmartDashboard/" + DASHBOARD_PREFIX;

  private static final String LIVE_SOURCE = "LIVE_CACHED_SPARK_TELEMETRY";
  private static final Map<String, Object> LAST_PUBLISHED = new HashMap<>();

  private SparkDeviceEvidence() {}

  /** Publishes cached live snapshots for every REV device in the configured CAN manifest. */
  public static void publish(List<DeviceEvidenceSnapshot> snapshots) {
    publishValues(valuesFor(snapshots, false));
  }

  /**
   * Invalidates dynamic evidence once when FMS suppression begins.
   *
   * <p>Previously cached numeric values are replaced with NaN so an old observation cannot look
   * live while the normal evidence publisher is intentionally disabled.
   */
  public static void publishSuppressedForFms() {
    publishValues(valuesFor(List.of(), true));
  }

  public static void publishUnavailable(String reason) {
    publishValues(unavailableValues(reason));
  }

  /** Dashboard key prefix; SmartDashboard places this below {@code /SmartDashboard}. */
  public static String dashboardPrefixForId(int canId) {
    return DASHBOARD_PREFIX + String.format("ID%02d/", canId);
  }

  /** Full NetworkTables path prefix used by Elastic and other read-only clients. */
  public static String networkTablesPrefixForId(int canId) {
    return "/SmartDashboard/" + dashboardPrefixForId(canId);
  }

  /**
   * Disambiguates an active output from an actual pending zero request.
   *
   * <p>{@link SparkOutputStopEvaluator.Status#CONFIRMED} is retained only after the evaluator has
   * checked the matching output epoch, successful zero, fresh post-zero telemetry, applied output,
   * velocity, and configured follower restoration.
   */
  static String stopState(
      SparkOutputStopEvaluator.Status evaluated,
      boolean zeroInFlight,
      boolean zeroRequired,
      boolean outputMayBeNonzero,
      boolean followerExpected,
      boolean leaderStopConfirmed) {
    Objects.requireNonNull(evaluated, "evaluated");
    if (evaluated == SparkOutputStopEvaluator.Status.CONFIRMED
        && followerExpected
        && !leaderStopConfirmed) {
      return "LEADER_STOP_NOT_CONFIRMED";
    }
    if (evaluated != SparkOutputStopEvaluator.Status.ZERO_PENDING) {
      return evaluated.name();
    }
    if (zeroInFlight) {
      return "ZERO_WRITE_IN_FLIGHT";
    }
    if (zeroRequired) {
      return "ZERO_REQUIRED_OR_RETRY_PENDING";
    }
    if (outputMayBeNonzero) {
      return "OUTPUT_MAY_BE_NONZERO";
    }
    return evaluated.name();
  }

  /** Pure topic/value projection used by production publishing and contract tests. */
  static Map<String, Object> valuesFor(
      List<DeviceEvidenceSnapshot> snapshots, boolean suppressedForFms) {
    List<DeviceEvidenceSnapshot> safeSnapshots = snapshots == null ? List.of() : snapshots;
    Map<Integer, DeviceEvidenceSnapshot> byCanId = new HashMap<>();
    Set<Integer> duplicateCanIds = new HashSet<>();
    for (DeviceEvidenceSnapshot snapshot : safeSnapshots) {
      if (snapshot == null) {
        continue;
      }
      if (byCanId.putIfAbsent(snapshot.canId(), snapshot) != null) {
        duplicateCanIds.add(snapshot.canId());
      }
    }

    Map<String, Object> values = new LinkedHashMap<>();
    for (Device device : configuredSparkDevices()) {
      DeviceEvidenceSnapshot snapshot = byCanId.get(device.canId());
      if (suppressedForFms) {
        addUnavailableValues(values, device, "SUPPRESSED_FMS", "SUPPRESSED_FMS");
      } else if (duplicateCanIds.contains(device.canId())) {
        addUnavailableValues(
            values,
            device,
            "DUPLICATE_REGISTERED_CAN_ID",
            "REGISTERED_CONTROLLER_CONFLICT");
      } else if (snapshot == null) {
        addUnavailableValues(
            values,
            device,
            "DEVICE_NOT_REGISTERED",
            "CONFIGURED_MANIFEST_WITHOUT_REGISTERED_CONTROLLER");
      } else {
        addLiveValues(values, device, snapshot);
      }
    }
    return Collections.unmodifiableMap(values);
  }

  static Map<String, Object> unavailableValues(String reason) {
    String safeReason = normalizedUnavailableReason(reason);
    Map<String, Object> values = new LinkedHashMap<>();
    for (Device device : configuredSparkDevices()) {
      addUnavailableValues(values, device, safeReason, safeReason);
    }
    return Collections.unmodifiableMap(values);
  }

  static List<Device> configuredSparkDevices() {
    return ConfiguredCanHardware.devices().stream()
        .filter(device -> device.vendor() == Vendor.REV)
        .toList();
  }

  private static void addLiveValues(
      Map<String, Object> values, Device device, DeviceEvidenceSnapshot snapshot) {
    String prefix = dashboardPrefixForId(device.canId());
    addManifestValues(values, prefix, device);
    values.put(prefix + "Source", LIVE_SOURCE);
    values.put(prefix + "Ready", snapshot.ready());
    values.put(prefix + "Reason", snapshot.reason());
    values.put(prefix + "Applied Output", snapshot.appliedOutput());
    values.put(prefix + "Current Amps", snapshot.currentAmps());
    values.put(prefix + "Velocity RPM", snapshot.velocityRpm());
    values.put(prefix + "Sample Age Seconds", snapshot.sampleAgeSeconds());
    values.put(prefix + "Sample Output Epoch", snapshot.sampleOutputEpoch());
    values.put(
        prefix + "Sample Matches Current Output Epoch",
        snapshot.sampleMatchesCurrentOutputEpoch());
    values.put(prefix + "Last Nonzero Setpoint API Returned OK", snapshot.lastRequestAccepted());
    values.put(prefix + "Last Nonzero Setpoint Epoch", snapshot.lastRequestEpoch());
    values.put(prefix + "Last Nonzero Setpoint Reason", snapshot.lastRequestReason());
    values.put(prefix + "Output Epoch", snapshot.outputEpoch());
    values.put(prefix + "Stop State", snapshot.stopState());
    values.put(prefix + "Snapshot", summary(LIVE_SOURCE, snapshot));
  }

  private static void addUnavailableValues(
      Map<String, Object> values, Device device, String reason, String source) {
    String prefix = dashboardPrefixForId(device.canId());
    addManifestValues(values, prefix, device);
    values.put(prefix + "Source", source);
    values.put(prefix + "Ready", false);
    values.put(prefix + "Reason", reason);
    values.put(prefix + "Applied Output", Double.NaN);
    values.put(prefix + "Current Amps", Double.NaN);
    values.put(prefix + "Velocity RPM", Double.NaN);
    values.put(prefix + "Sample Age Seconds", Double.NaN);
    values.put(prefix + "Sample Output Epoch", -1L);
    values.put(prefix + "Sample Matches Current Output Epoch", false);
    values.put(prefix + "Last Nonzero Setpoint API Returned OK", false);
    values.put(prefix + "Last Nonzero Setpoint Epoch", -1L);
    values.put(prefix + "Last Nonzero Setpoint Reason", reason);
    values.put(prefix + "Output Epoch", -1L);
    values.put(prefix + "Stop State", "UNAVAILABLE_" + reason);
    values.put(
        prefix + "Snapshot",
        "source=" + source + " ready=false reason=" + reason
            + " telemetry=UNAVAILABLE stop=UNAVAILABLE_" + reason);
  }

  private static void addManifestValues(Map<String, Object> values, String prefix, Device device) {
    values.put(prefix + "Label", device.label());
    values.put(prefix + "Vendor", device.vendor().name());
    values.put(prefix + "Type", device.type().name());
    values.put(prefix + "Role", device.role().name());
    values.put(prefix + "Dependency CAN IDs", device.dependencyCanIds().toString());
  }

  private static String summary(String source, DeviceEvidenceSnapshot snapshot) {
    return "source=" + source
        + " ready=" + snapshot.ready()
        + " reason=" + snapshot.reason()
        + " applied=" + snapshot.appliedOutput()
        + " currentAmps=" + snapshot.currentAmps()
        + " velocityRpm=" + snapshot.velocityRpm()
        + " sampleAgeSeconds=" + snapshot.sampleAgeSeconds()
        + " sampleOutputEpoch=" + snapshot.sampleOutputEpoch()
        + " sampleMatchesCurrentOutputEpoch=" + snapshot.sampleMatchesCurrentOutputEpoch()
        + " lastNonzeroSetpointApiReturnedOk=" + snapshot.lastRequestAccepted()
        + " lastNonzeroSetpointEpoch=" + snapshot.lastRequestEpoch()
        + " lastNonzeroSetpointReason=" + snapshot.lastRequestReason()
        + " outputEpoch=" + snapshot.outputEpoch()
        + " stop=" + snapshot.stopState();
  }

  private static synchronized void publishValues(Map<String, Object> values) {
    for (Map.Entry<String, Object> entry : values.entrySet()) {
      String key = entry.getKey();
      Object value = entry.getValue();
      if (sameValue(LAST_PUBLISHED.get(key), value)) {
        continue;
      }
      if (value instanceof Boolean booleanValue) {
        SmartDashboard.putBoolean(key, booleanValue);
      } else if (value instanceof Number numberValue) {
        SmartDashboard.putNumber(key, numberValue.doubleValue());
      } else {
        SmartDashboard.putString(key, String.valueOf(value));
      }
      LAST_PUBLISHED.put(key, value);
    }
  }

  private static boolean sameValue(Object previous, Object current) {
    if (previous instanceof Double previousDouble && current instanceof Double currentDouble) {
      return Double.doubleToLongBits(previousDouble) == Double.doubleToLongBits(currentDouble);
    }
    return Objects.equals(previous, current);
  }

  private static String normalizedUnavailableReason(String reason) {
    if (reason == null || reason.isBlank()) {
      return "UNAVAILABLE_UNKNOWN";
    }
    return reason.trim().replaceAll("[^A-Za-z0-9_-]+", "_");
  }
}
