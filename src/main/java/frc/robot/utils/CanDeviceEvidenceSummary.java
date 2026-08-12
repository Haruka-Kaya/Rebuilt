package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ConfiguredCanHardware;
import frc.robot.constants.ConfiguredCanHardware.Vendor;
import frc.robot.utils.CtreDeviceEvidence.Snapshot;
import frc.robot.utils.SparkMAXContainer.DeviceEvidenceSnapshot;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/** Compact, manifest-driven operator view of all 23 configured CAN addresses. */
public final class CanDeviceEvidenceSummary {
  private static final int MAX_COMPACT_FIELD_LENGTH = 32;

  public static final String SCOPE_KEY = "Hardware/Device Evidence Scope";
  public static final String SUMMARY_KEY = "Hardware/Device Evidence Summary";
  public static final String SCOPE =
      "REPORTED_CACHED_STATUS_ONLY\n"
          + "NOT_PHYSICAL_ID_UNIQUENESS_OR_MOTION_OR_CALIBRATION_PROOF\n"
          + "EPOCH_MATCH_IS_NOT_POST_ZERO_PROOF_USE_STOP_STATE\n"
          + "CTRE_REQUEST_ACCEPTANCE_NOT_OBSERVED_PER_DEVICE\n"
          + "DETAILS_UNDER_Hardware/Device/IDxx";

  private CanDeviceEvidenceSummary() {}

  public static void publish(
      List<DeviceEvidenceSnapshot> sparkSnapshots, List<Snapshot> ctreSnapshots) {
    SmartDashboard.putString(SCOPE_KEY, SCOPE);
    SmartDashboard.putString(SUMMARY_KEY, format(sparkSnapshots, ctreSnapshots));
  }

  public static void publishSuppressedForFms() {
    SmartDashboard.putString(SCOPE_KEY, SCOPE);
    SmartDashboard.putString(SUMMARY_KEY, "SUPPRESSED_FMS");
  }

  public static void publishUnavailable(String reason) {
    SmartDashboard.putString(SCOPE_KEY, SCOPE);
    SmartDashboard.putString(
        SUMMARY_KEY,
        compact(reason == null || reason.isBlank() ? "UNAVAILABLE_UNKNOWN" : reason));
  }

  static String format(
      List<DeviceEvidenceSnapshot> sparkSnapshots, List<Snapshot> ctreSnapshots) {
    Map<Integer, DeviceEvidenceSnapshot> sparkById = new HashMap<>();
    Set<Integer> duplicateSparkIds = new HashSet<>();
    for (DeviceEvidenceSnapshot snapshot : sparkSnapshots == null
        ? List.<DeviceEvidenceSnapshot>of() : sparkSnapshots) {
      if (snapshot != null) {
        if (sparkById.putIfAbsent(snapshot.canId(), snapshot) != null) {
          duplicateSparkIds.add(snapshot.canId());
        }
      }
    }
    Map<Integer, Snapshot> ctreById = new HashMap<>();
    Set<Integer> duplicateCtreIds = new HashSet<>();
    for (Snapshot snapshot : ctreSnapshots == null ? List.<Snapshot>of() : ctreSnapshots) {
      if (snapshot != null) {
        if (ctreById.putIfAbsent(snapshot.canId(), snapshot) != null) {
          duplicateCtreIds.add(snapshot.canId());
        }
      }
    }

    StringBuilder summary = new StringBuilder();
    for (var device : ConfiguredCanHardware.devices()) {
      if (!summary.isEmpty()) {
        summary.append('\n');
      }
      summary.append("ID").append(device.canId())
          .append(' ').append(device.vendor())
          .append(' ').append(device.label());
      if (!device.dependencyCanIds().isEmpty()) {
        summary.append(" deps=").append(device.dependencyCanIds());
      }
      if (device.vendor() == Vendor.REV) {
        if (duplicateSparkIds.contains(device.canId())) {
          summary.append(" ready=false reason=DUPLICATE_REGISTERED_CAN_ID");
        } else {
          appendSpark(summary, sparkById.get(device.canId()));
        }
      } else {
        if (duplicateCtreIds.contains(device.canId())) {
          summary.append(" ready=false reason=DUPLICATE_DEVICE_EVIDENCE");
        } else {
          appendCtre(summary, ctreById.get(device.canId()));
        }
      }
    }
    return summary.toString();
  }

  private static void appendSpark(StringBuilder result, DeviceEvidenceSnapshot snapshot) {
    if (snapshot == null) {
      result.append(" ready=false reason=DEVICE_NOT_REGISTERED");
      return;
    }
    result.append(" ready=").append(snapshot.ready())
        .append(" reason=").append(compact(snapshot.reason()))
        .append(" age=").append(value(snapshot.sampleAgeSeconds())).append('s')
        .append(" epoch=").append(snapshot.sampleOutputEpoch())
        .append('/').append(snapshot.outputEpoch())
        .append(" match=").append(snapshot.sampleMatchesCurrentOutputEpoch())
        .append(" stop=").append(compact(snapshot.stopState()));
  }

  private static void appendCtre(StringBuilder result, Snapshot snapshot) {
    if (snapshot == null) {
      result.append(" ready=false reason=DEVICE_EVIDENCE_NOT_CAPTURED");
      return;
    }
    result.append(" ready=").append(snapshot.ready())
        .append(" reason=").append(compact(snapshot.reason()))
        .append(" age=").append(value(snapshot.maximumSampleAgeSeconds())).append('s')
        .append(" diag=").append(snapshot.diagnosticTelemetryReady())
        .append('/').append(compact(snapshot.diagnosticReason()));
  }

  private static String compact(String value) {
    String normalized = value == null || value.isBlank()
        ? "N/A"
        : value.trim().replaceAll("\\s+", " ");
    if (normalized.length() <= MAX_COMPACT_FIELD_LENGTH) {
      return normalized;
    }
    return normalized.substring(0, MAX_COMPACT_FIELD_LENGTH - 3) + "...";
  }

  private static String value(double value) {
    return Double.isFinite(value) ? Double.toString(value) : "N/A";
  }
}
