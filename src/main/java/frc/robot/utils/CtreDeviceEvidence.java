package frc.robot.utils;

import java.util.List;
import java.util.Objects;
import java.util.OptionalDouble;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ConfiguredCanHardware;

/** Pure, immutable evidence captured from one configured CTRE device. */
public final class CtreDeviceEvidence {
  public static final String DASHBOARD_PREFIX = "Hardware/Device/";
  public enum Metric {
    CONTROL_POSITION_ROTATIONS,
    CONTROL_VELOCITY_ROTATIONS_PER_SECOND,
    ROTOR_POSITION_ROTATIONS,
    ROTOR_VELOCITY_ROTATIONS_PER_SECOND,
    ENCODER_POSITION_ROTATIONS,
    ENCODER_VELOCITY_ROTATIONS_PER_SECOND,
    ABSOLUTE_POSITION_ROTATIONS,
    YAW_DEGREES,
    ANGULAR_VELOCITY_DEGREES_PER_SECOND,
    DUTY_CYCLE,
    STATOR_CURRENT_AMPS,
    MOTOR_VOLTAGE_VOLTS
  }

  /** One nonblocking Phoenix status-signal observation. */
  public record SignalObservation(
      String name,
      Metric metric,
      boolean fresh,
      String reason,
      double value,
      double sampleTimestampSeconds,
      double progressTimestampSeconds,
      double sampleAgeSeconds) {
    public SignalObservation {
      Objects.requireNonNull(name, "name");
      Objects.requireNonNull(metric, "metric");
      Objects.requireNonNull(reason, "reason");
      if (name.isBlank() || reason.isBlank()) {
        throw new IllegalArgumentException("signal name and reason must not be blank");
      }
    }
  }

  /** Operator-facing snapshot; {@code ready} is specific to this CAN ID, not a group copy. */
  public record Snapshot(
      int canId,
      String label,
      String role,
      boolean ready,
      String reason,
      boolean diagnosticTelemetryReady,
      String diagnosticReason,
      double capturedAtSeconds,
      List<SignalObservation> observations) {
    public Snapshot {
      if (canId < 0 || canId > 62) {
        throw new IllegalArgumentException("CAN ID must be within 0..62");
      }
      Objects.requireNonNull(label, "label");
      Objects.requireNonNull(role, "role");
      Objects.requireNonNull(reason, "reason");
      Objects.requireNonNull(diagnosticReason, "diagnosticReason");
      observations = List.copyOf(observations);
      if (label.isBlank() || role.isBlank() || reason.isBlank() || diagnosticReason.isBlank()) {
        throw new IllegalArgumentException("snapshot text fields must not be blank");
      }
    }

    public OptionalDouble value(Metric metric) {
      return observations.stream()
          .filter(observation -> observation.metric() == metric)
          .filter(SignalObservation::fresh)
          .mapToDouble(SignalObservation::value)
          .findFirst();
    }

    public OptionalDouble sampleTimestamp(Metric metric) {
      return observations.stream()
          .filter(observation -> observation.metric() == metric)
          .filter(SignalObservation::fresh)
          .mapToDouble(SignalObservation::sampleTimestampSeconds)
          .findFirst();
    }

    public double maximumSampleAgeSeconds() {
      return observations.stream()
          .mapToDouble(SignalObservation::sampleAgeSeconds)
          .filter(Double::isFinite)
          .max()
          .orElse(Double.NaN);
    }
  }

  private CtreDeviceEvidence() {}

  /** Publishes only the supplied cached primitives; this method performs no Phoenix I/O. */
  public static void publish(List<Snapshot> snapshots) {
    publishValues(valuesFor(snapshots, false));
  }

  /** Invalidates the 13 CTRE device topics when live publishing is intentionally suppressed. */
  public static void publishSuppressedForFms() {
    publishValues(valuesFor(List.of(), true));
  }

  public static void publishUnavailable(String reason) {
    String safeReason = normalize(reason);
    Map<String, Object> values = new LinkedHashMap<>();
    for (var device : ConfiguredCanHardware.devices()) {
      if (device.vendor() == ConfiguredCanHardware.Vendor.CTRE) {
        addUnavailableValues(values, device, safeReason);
      }
    }
    publishValues(values);
  }

  static Map<String, Object> valuesFor(List<Snapshot> snapshots, boolean suppressedForFms) {
    Map<Integer, Snapshot> byCanId = new HashMap<>();
    Set<Integer> duplicateCanIds = new HashSet<>();
    for (Snapshot snapshot : snapshots == null ? List.<Snapshot>of() : snapshots) {
      if (snapshot != null && byCanId.putIfAbsent(snapshot.canId(), snapshot) != null) {
        duplicateCanIds.add(snapshot.canId());
      }
    }

    Map<String, Object> values = new LinkedHashMap<>();
    for (var device : ConfiguredCanHardware.devices()) {
      if (device.vendor() != ConfiguredCanHardware.Vendor.CTRE) {
        continue;
      }
      Snapshot snapshot = byCanId.get(device.canId());
      String reason = suppressedForFms
          ? "SUPPRESSED_FMS"
          : duplicateCanIds.contains(device.canId())
              ? "DUPLICATE_DEVICE_EVIDENCE"
              : snapshot == null ? "DEVICE_EVIDENCE_NOT_CAPTURED" : null;
      if (reason == null) {
        addLiveValues(values, device, snapshot);
      } else {
        addUnavailableValues(values, device, reason);
      }
    }
    return Collections.unmodifiableMap(values);
  }

  private static void addLiveValues(
      Map<String, Object> values,
      ConfiguredCanHardware.Device device,
      Snapshot snapshot) {
    String prefix = dashboardPrefixForId(device.canId());
    addManifestValues(values, prefix, device);
    values.put(prefix + "Source", "LIVE_CACHED_CTRE_STATUS_SIGNALS");
    values.put(prefix + "Ready", snapshot.ready());
    values.put(prefix + "Reason", snapshot.reason());
    values.put(prefix + "Diagnostic Ready", snapshot.diagnosticTelemetryReady());
    values.put(prefix + "Diagnostic Reason", snapshot.diagnosticReason());
    values.put(prefix + "Sample Age Seconds", snapshot.maximumSampleAgeSeconds());
    values.put(prefix + "Applied Output", snapshot.value(Metric.DUTY_CYCLE).orElse(Double.NaN));
    values.put(
        prefix + "Motor Voltage Volts",
        snapshot.value(Metric.MOTOR_VOLTAGE_VOLTS).orElse(Double.NaN));
    values.put(
        prefix + "Rotor Position Rotations",
        snapshot.value(Metric.ROTOR_POSITION_ROTATIONS).orElse(Double.NaN));
    values.put(
        prefix + "Control Position Rotations",
        snapshot.value(Metric.CONTROL_POSITION_ROTATIONS).orElse(Double.NaN));
    values.put(
        prefix + "Absolute Position Rotations",
        snapshot.value(Metric.ABSOLUTE_POSITION_ROTATIONS).orElse(Double.NaN));
    values.put(
        prefix + "Encoder Position Rotations",
        snapshot.value(Metric.ENCODER_POSITION_ROTATIONS).orElse(Double.NaN));
    values.put(
        prefix + "Encoder Velocity RPS",
        snapshot.value(Metric.ENCODER_VELOCITY_ROTATIONS_PER_SECOND).orElse(Double.NaN));
    values.put(prefix + "Yaw Degrees", snapshot.value(Metric.YAW_DEGREES).orElse(Double.NaN));
    values.put(
        prefix + "Angular Velocity Degrees Per Second",
        snapshot.value(Metric.ANGULAR_VELOCITY_DEGREES_PER_SECOND).orElse(Double.NaN));
    values.put(
        prefix + "Current Amps",
        snapshot.diagnosticTelemetryReady()
            ? snapshot.value(Metric.STATOR_CURRENT_AMPS).orElse(Double.NaN)
            : Double.NaN);
    values.put(
        prefix + "Velocity RPM",
        snapshot.diagnosticTelemetryReady()
            ? snapshot.value(Metric.ROTOR_VELOCITY_ROTATIONS_PER_SECOND)
                .orElse(Double.NaN) * 60.0
            : Double.NaN);
    values.put(prefix + "Last Nonzero Setpoint API Returned OK", "NOT_OBSERVED_BY_DEVICE_API");
    values.put(prefix + "Stop State", "NOT_OBSERVED_PER_DEVICE");
    values.put(prefix + "Snapshot", summary(snapshot));
  }

  private static void addUnavailableValues(
      Map<String, Object> values,
      ConfiguredCanHardware.Device device,
      String reason) {
    String prefix = dashboardPrefixForId(device.canId());
    addManifestValues(values, prefix, device);
    values.put(prefix + "Source", reason);
    values.put(prefix + "Ready", false);
    values.put(prefix + "Reason", reason);
    values.put(prefix + "Diagnostic Ready", false);
    values.put(prefix + "Diagnostic Reason", reason);
    values.put(prefix + "Sample Age Seconds", Double.NaN);
    values.put(prefix + "Applied Output", Double.NaN);
    values.put(prefix + "Motor Voltage Volts", Double.NaN);
    values.put(prefix + "Rotor Position Rotations", Double.NaN);
    values.put(prefix + "Control Position Rotations", Double.NaN);
    values.put(prefix + "Absolute Position Rotations", Double.NaN);
    values.put(prefix + "Encoder Position Rotations", Double.NaN);
    values.put(prefix + "Encoder Velocity RPS", Double.NaN);
    values.put(prefix + "Yaw Degrees", Double.NaN);
    values.put(prefix + "Angular Velocity Degrees Per Second", Double.NaN);
    values.put(prefix + "Current Amps", Double.NaN);
    values.put(prefix + "Velocity RPM", Double.NaN);
    values.put(prefix + "Last Nonzero Setpoint API Returned OK", "NOT_OBSERVED_BY_DEVICE_API");
    values.put(prefix + "Stop State", "UNAVAILABLE_" + reason);
    values.put(prefix + "Snapshot", "source=" + reason + " ready=false reason=" + reason);
  }

  private static void addManifestValues(
      Map<String, Object> values,
      String prefix,
      ConfiguredCanHardware.Device device) {
    values.put(prefix + "Label", device.label());
    values.put(prefix + "Vendor", "CTRE");
    values.put(prefix + "Type", device.type().name());
    values.put(prefix + "Role", device.role().name());
    values.put(prefix + "Dependency CAN IDs", device.dependencyCanIds().toString());
  }

  private static void publishValues(Map<String, Object> values) {
    for (Map.Entry<String, Object> entry : values.entrySet()) {
      Object value = entry.getValue();
      if (value instanceof Boolean booleanValue) {
        SmartDashboard.putBoolean(entry.getKey(), booleanValue);
      } else if (value instanceof Number numberValue) {
        SmartDashboard.putNumber(entry.getKey(), numberValue.doubleValue());
      } else {
        SmartDashboard.putString(entry.getKey(), String.valueOf(value));
      }
    }
  }

  public static String dashboardPrefixForId(int canId) {
    return DASHBOARD_PREFIX + String.format("ID%02d/", canId);
  }

  private static String summary(Snapshot snapshot) {
    StringBuilder result = new StringBuilder()
        .append("ready=").append(snapshot.ready())
        .append(" reason=").append(snapshot.reason())
        .append(" diagnosticReady=").append(snapshot.diagnosticTelemetryReady())
        .append(" diagnosticReason=").append(snapshot.diagnosticReason());
    for (SignalObservation observation : snapshot.observations()) {
      result.append(' ').append(observation.name()).append('=');
      if (observation.fresh()) {
        result.append(observation.value());
      } else {
        result.append("N/A/").append(observation.reason());
      }
    }
    return result.toString();
  }

  /** Converts raw Phoenix fields to one stable, fail-closed observation. */
  public static SignalObservation evaluateSignal(
      String name,
      Metric metric,
      boolean statusOk,
      String statusName,
      boolean timestampValid,
      double sampleTimestampSeconds,
      double progressTimestampSeconds,
      double sampleAgeSeconds,
      double value,
      double maximumAgeSeconds) {
    String reason;
    if (!statusOk) {
      reason = "STATUS_" + normalize(statusName);
    } else if (!timestampValid) {
      reason = "TIMESTAMP_INVALID";
    } else if (!Double.isFinite(sampleTimestampSeconds)) {
      reason = "SAMPLE_TIME_NONFINITE";
    } else if (!Double.isFinite(progressTimestampSeconds)) {
      reason = "PROGRESS_TIMESTAMP_INVALID";
    } else if (!Double.isFinite(sampleAgeSeconds) || sampleAgeSeconds < 0.0) {
      reason = "SAMPLE_AGE_INVALID";
    } else if (!Double.isFinite(maximumAgeSeconds) || maximumAgeSeconds < 0.0) {
      reason = "MAXIMUM_AGE_INVALID";
    } else if (sampleAgeSeconds > maximumAgeSeconds) {
      reason = "STALE";
    } else if (!Double.isFinite(value)) {
      reason = "VALUE_NONFINITE";
    } else {
      reason = "OK";
    }
    return new SignalObservation(
        name,
        metric,
        "OK".equals(reason),
        reason,
        value,
        sampleTimestampSeconds,
        progressTimestampSeconds,
        sampleAgeSeconds);
  }

  public static String firstFailureReason(List<SignalObservation> observations) {
    if (observations == null || observations.isEmpty()) {
      return "NO_SIGNALS";
    }
    return observations.stream()
        .filter(observation -> !observation.fresh())
        .map(observation -> observation.name() + "/" + observation.reason())
        .findFirst()
        .orElse("OK");
  }

  public static boolean allFresh(List<SignalObservation> observations) {
    return observations != null
        && !observations.isEmpty()
        && observations.stream().allMatch(SignalObservation::fresh);
  }

  private static String normalize(String value) {
    if (value == null || value.isBlank()) {
      return "UNKNOWN";
    }
    return value.trim().replaceAll("[^A-Za-z0-9_-]+", "_");
  }
}
