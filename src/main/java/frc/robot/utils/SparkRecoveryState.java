package frc.robot.utils;

/** Pure per-device state machine for asynchronous SPARK discovery and configuration. */
final class SparkRecoveryState {
  static final double INITIAL_GRACE_SECONDS = 0.50;

  enum ServiceAction {
    NONE,
    PROBE_AND_APPLY_RESET,
    APPLY_UPDATE
  }

  private enum Health {
    DISCOVERING,
    READY,
    RETRY_WAIT
  }

  private Health health = Health.DISCOVERING;
  private long desiredRevision;
  private long appliedRevision = -1;
  private boolean resetRequired = true;
  private boolean operationInFlight;
  private boolean initialPersistenceAttempted;
  private long initialPersistenceConfirmedRevision = -1;
  private int consecutiveFailures;
  private double nextAttemptAt;
  private String lastError = "not configured";

  SparkRecoveryState(double nowSeconds, long initialDesiredRevision) {
    desiredRevision = initialDesiredRevision;
    nextAttemptAt = nowSeconds + INITIAL_GRACE_SECONDS;
  }

  void desiredRevisionChanged(long revision, double nowSeconds) {
    desiredRevision = revision;
    if (health == Health.READY && !operationInFlight) {
      nextAttemptAt = nowSeconds;
    }
  }

  ServiceAction peekServiceAction(double nowSeconds) {
    if (operationInFlight || nowSeconds < nextAttemptAt) {
      return ServiceAction.NONE;
    }
    if (health != Health.READY || resetRequired) {
      return ServiceAction.PROBE_AND_APPLY_RESET;
    }
    if (appliedRevision != desiredRevision) {
      return ServiceAction.APPLY_UPDATE;
    }
    return ServiceAction.NONE;
  }

  ServiceAction beginService(double nowSeconds) {
    ServiceAction action = peekServiceAction(nowSeconds);
    if (action != ServiceAction.NONE) {
      operationInFlight = true;
    }
    return action;
  }

  void operationSucceeded(long completedRevision, double nowSeconds) {
    operationInFlight = false;
    health = Health.READY;
    appliedRevision = completedRevision;
    resetRequired = false;
    consecutiveFailures = 0;
    nextAttemptAt = nowSeconds;
    lastError = "ok";
  }

  boolean shouldPersist(ServiceAction action) {
    return action == ServiceAction.PROBE_AND_APPLY_RESET && !initialPersistenceAttempted;
  }

  void persistenceAttempted() {
    initialPersistenceAttempted = true;
  }

  void persistenceSucceeded(long revision) {
    initialPersistenceConfirmedRevision = revision;
  }

  void operationCancelled(double nowSeconds) {
    operationInFlight = false;
    nextAttemptAt = nowSeconds;
  }

  void operationFailed(double nowSeconds, String error) {
    operationInFlight = false;
    health = Health.RETRY_WAIT;
    appliedRevision = -1;
    resetRequired = true;
    consecutiveFailures++;
    nextAttemptAt = nowSeconds + retryDelaySeconds(consecutiveFailures);
    lastError = error;
  }

  void resetDetected(double nowSeconds) {
    operationInFlight = false;
    health = Health.RETRY_WAIT;
    appliedRevision = -1;
    resetRequired = true;
    nextAttemptAt = nowSeconds;
    lastError = "controller reset";
  }

  boolean isConfigurationReady() {
    return health == Health.READY
        && !operationInFlight
        && !resetRequired
        && appliedRevision == desiredRevision;
  }

  boolean isOperationInFlight() {
    return operationInFlight;
  }

  long getDesiredRevision() {
    return desiredRevision;
  }

  int getConsecutiveFailures() {
    return consecutiveFailures;
  }

  String getSummary() {
    String persistence = "";
    if (initialPersistenceAttempted) {
      if (initialPersistenceConfirmedRevision < 0) {
        persistence = "/PERSIST_UNCONFIRMED";
      } else if (initialPersistenceConfirmedRevision != desiredRevision) {
        persistence = "/PERSIST_STALE";
      }
    }
    if (isConfigurationReady()) {
      return "READY" + persistence;
    }
    if (operationInFlight) {
      return "CONFIGURING" + persistence;
    }
    if (health == Health.DISCOVERING) {
      return "DISCOVERING" + persistence;
    }
    if (health == Health.READY) {
      return "CONFIG_PENDING" + persistence;
    }
    return "OFFLINE(" + lastError + ")" + persistence;
  }

  static double retryDelaySeconds(int consecutiveFailures) {
    if (consecutiveFailures <= 1) {
      return 0.5;
    }
    if (consecutiveFailures == 2) {
      return 1.0;
    }
    if (consecutiveFailures == 3) {
      return 2.0;
    }
    if (consecutiveFailures == 4) {
      return 4.0;
    }
    return 5.0;
  }
}
