package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ConfiguredOperatorActions;
import frc.robot.constants.ConfiguredOperatorActions.Action;
import frc.robot.constants.ConfiguredOperatorActions.ActionSpec;
import java.util.Arrays;
import java.util.Collection;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Collectors;

/** Change-only operator evidence for requested actions and their exact current block reasons. */
public final class OperatorActionEvidence {
  public static final String PREFIX = "Operator Actions/";
  public static final String CONFIGURED_KEY = PREFIX + "Configured";
  public static final String CURRENT_STATE_KEY = PREFIX + "Current State";
  public static final String CURRENT_REASON_KEY = PREFIX + "Current Reason";
  public static final String EVIDENCE_KEY = PREFIX + "Evidence";

  public enum State {
    PRESSED,
    /**
     * The software accepted or issued the action request. This is not proof that a mechanism
     * physically moved or reached its target.
     */
    ACTIVE,
    BLOCKED,
    STOPPED,
    COMPLETED
  }

  @FunctionalInterface
  interface StringPublisher {
    void publish(String key, String value);
  }

  /** Immutable evidence snapshot. Reason order preserves the caller's priority order. */
  public record Snapshot(Action action, State state, List<String> reasons, long sequence) {
    public Snapshot {
      Objects.requireNonNull(action, "action");
      Objects.requireNonNull(state, "state");
      reasons = List.copyOf(Objects.requireNonNull(reasons, "reasons"));
    }

    /** Stable, human-readable representation used by the dashboard's current-reason topic. */
    public String reason() {
      return String.join(" + ", reasons);
    }
  }

  private final StringPublisher publisher;
  private final Map<String, String> publishedValues = new HashMap<>();
  private final Map<Action, Snapshot> snapshots = new EnumMap<>(Action.class);
  private long sequence;

  public OperatorActionEvidence() {
    this(SmartDashboard::putString);
  }

  OperatorActionEvidence(StringPublisher publisher) {
    this.publisher = Objects.requireNonNull(publisher, "publisher");
    publishChanged(CONFIGURED_KEY, ConfiguredOperatorActions.configuredSummary());
    for (ActionSpec actionSpec : ConfiguredOperatorActions.specs()) {
      Action action = actionSpec.action();
      publishChanged(outputCanIdsKey(action), actionSpec.outputCanIds().toString());
      publishChanged(dependencyCanIdsKey(action), actionSpec.dependencyCanIds().toString());
      Snapshot initial = new Snapshot(action, State.STOPPED, List.of("NOT_PRESSED"), 0);
      snapshots.put(action, initial);
      publishChanged(stateKey(action), initial.state().name());
      publishChanged(reasonKey(action), initial.reason());
    }
    publishChanged(CURRENT_STATE_KEY, "NONE STOPPED");
    publishChanged(CURRENT_REASON_KEY, "NOT_PRESSED");
    publishChanged(EVIDENCE_KEY, aggregateEvidence());
  }

  /** Records that an operator request reached its command/input gate. */
  public synchronized Snapshot requested(Action action) {
    return update(action, State.PRESSED, List.of("INPUT_ACCEPTED"));
  }

  public synchronized Snapshot pressed(Action action, String... reasons) {
    return update(action, State.PRESSED, normalizeReasons(reasons));
  }

  /**
   * Records an accepted/issued action request. ACTIVE does not prove physical mechanism motion.
   */
  public synchronized Snapshot active(Action action, String... reasons) {
    return update(action, State.ACTIVE, normalizeReasons(reasons));
  }

  /**
   * Records an accepted/issued action request. ACTIVE does not prove physical mechanism motion.
   */
  public synchronized Snapshot active(Action action, Collection<String> reasons) {
    return update(action, State.ACTIVE, normalizeReasons(reasons));
  }

  public synchronized Snapshot blocked(Action action, String... reasons) {
    return update(action, State.BLOCKED, normalizeReasons(reasons));
  }

  public synchronized Snapshot blocked(Action action, Collection<String> reasons) {
    return update(action, State.BLOCKED, normalizeReasons(reasons));
  }

  public synchronized Snapshot stopped(Action action, String reason) {
    return update(action, State.STOPPED, normalizeReasons(new String[] {reason}));
  }

  public synchronized Snapshot stopped(Action action, Collection<String> reasons) {
    return update(action, State.STOPPED, normalizeReasons(reasons));
  }

  public synchronized Snapshot stopped(Action action, boolean interrupted) {
    return stopped(action, interrupted ? "INTERRUPTED" : "INPUT_RELEASED");
  }

  /**
   * Records a command end without overwriting an input predicate that already observed release.
   * WPILib reports a normal {@code whileTrue} release as interrupted, so that boolean alone is not
   * precise enough to call an operator release an interruption.
   */
  public synchronized Snapshot commandEnded(Action action, boolean interrupted) {
    Snapshot previous = snapshot(action);
    if (previous != null
        && (previous.state() == State.STOPPED || previous.state() == State.BLOCKED)) {
      return previous;
    }
    return stopped(
        action,
        interrupted ? "TRIGGER_RELEASED_OR_COMMAND_INTERRUPTED" : "COMMAND_COMPLETED");
  }

  public synchronized Snapshot completed(Action action, String... reasons) {
    return update(action, State.COMPLETED, normalizeReasons(reasons));
  }

  /** Clears every potentially stale ACTIVE/BLOCKED action after a lifecycle-wide output stop. */
  public synchronized void allStopped(String reason) {
    for (Action action : ConfiguredOperatorActions.actions()) {
      update(action, State.STOPPED, normalizeReasons(new String[] {reason}));
    }
    publishChanged(CURRENT_STATE_KEY, "ALL STOPPED");
    publishChanged(CURRENT_REASON_KEY, normalizeReason(reason));
    publishChanged(EVIDENCE_KEY, aggregateEvidence());
  }

  /** @deprecated Waiting is an ACTIVE request with an explicit waiting reason. */
  @Deprecated(forRemoval = false)
  public synchronized Snapshot waiting(Action action, String reason) {
    return active(action, reason);
  }

  /** @deprecated Partial acceptance is ACTIVE with a reason describing the accepted subset. */
  @Deprecated(forRemoval = false)
  public synchronized Snapshot partial(Action action, String reason) {
    return active(action, reason);
  }

  /** @deprecated Faults block an action and should include the exact fault reason. */
  @Deprecated(forRemoval = false)
  public synchronized Snapshot faulted(Action action, String reason) {
    return blocked(action, reason);
  }

  public synchronized Snapshot snapshot(Action action) {
    return snapshots.get(Objects.requireNonNull(action, "action"));
  }

  private Snapshot update(Action action, State state, List<String> reasons) {
    Objects.requireNonNull(action, "action");
    Objects.requireNonNull(state, "state");
    Snapshot previous = snapshots.get(action);
    if (previous != null && previous.state() == state && previous.reasons().equals(reasons)) {
      return previous;
    }
    Snapshot next = new Snapshot(action, state, reasons, ++sequence);
    snapshots.put(action, next);
    publishChanged(stateKey(action), state.name());
    publishChanged(reasonKey(action), next.reason());
    publishChanged(CURRENT_STATE_KEY, action.name() + " " + state.name());
    publishChanged(CURRENT_REASON_KEY, next.reason());
    publishChanged(EVIDENCE_KEY, aggregateEvidence());
    return next;
  }

  private String aggregateEvidence() {
    return ConfiguredOperatorActions.actions().stream()
        .map(action -> {
          Snapshot snapshot = snapshots.get(action);
          return action.name() + "=" + snapshot.state().name() + "[" + snapshot.reason() + "]";
        })
        .collect(Collectors.joining("; "));
  }

  private void publishChanged(String key, String value) {
    if (Objects.equals(publishedValues.get(key), value) && publishedValues.containsKey(key)) {
      return;
    }
    publishedValues.put(key, value);
    publisher.publish(key, value);
  }

  private static List<String> normalizeReasons(String... reasons) {
    return normalizeReasons(reasons == null ? List.of() : Arrays.asList(reasons));
  }

  private static List<String> normalizeReasons(Collection<String> reasons) {
    LinkedHashSet<String> normalized = new LinkedHashSet<>();
    if (reasons != null) {
      for (String reason : reasons) {
        normalized.add(normalizeReason(reason));
      }
    }
    if (normalized.isEmpty()) {
      normalized.add("UNSPECIFIED");
    }
    return List.copyOf(normalized);
  }

  private static String normalizeReason(String reason) {
    if (reason == null || reason.isBlank()) {
      return "UNSPECIFIED";
    }
    String normalized = reason.trim()
        .replaceAll("[^A-Za-z0-9]+", "_")
        .replaceAll("^_+|_+$", "")
        .toUpperCase(Locale.ROOT);
    return normalized.isBlank() ? "UNSPECIFIED" : normalized;
  }

  public static String stateKey(Action action) {
    return PREFIX + action.topicKey() + "/State";
  }

  public static String reasonKey(Action action) {
    return PREFIX + action.topicKey() + "/Reason";
  }

  public static String outputCanIdsKey(Action action) {
    return PREFIX + action.topicKey() + "/Output CAN IDs";
  }

  public static String dependencyCanIdsKey(Action action) {
    return PREFIX + action.topicKey() + "/Dependency CAN IDs";
  }
}
