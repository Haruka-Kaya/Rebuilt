package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.utils.SparkRecoveryState.ServiceAction;

class SparkRecoveryStateTest {
  @Test
  void persistsOnlyTheFirstFullConfigurationInThisProcess() {
    SparkRecoveryState state = new SparkRecoveryState(10.0, 4);

    assertEquals(ServiceAction.NONE, state.peekServiceAction(10.49));
    ServiceAction initialAction = state.beginService(10.50);
    assertEquals(ServiceAction.PROBE_AND_APPLY_RESET, initialAction);
    assertTrue(state.shouldPersist(initialAction));
    assertEquals(ServiceAction.NONE, state.peekServiceAction(20.0));

    state.persistenceAttempted();
    state.persistenceSucceeded(4);
    state.operationSucceeded(4, 10.60);

    assertTrue(state.isConfigurationReady());
    assertEquals(ServiceAction.NONE, state.peekServiceAction(100.0));

    state.resetDetected(101.0);
    ServiceAction reconnectAction = state.beginService(101.0);
    assertEquals(ServiceAction.PROBE_AND_APPLY_RESET, reconnectAction);
    assertFalse(state.shouldPersist(reconnectAction));
  }

  @Test
  void configChangeBlocksOutputUntilLatestRevisionIsApplied() {
    SparkRecoveryState state = readyState(3);

    state.desiredRevisionChanged(4, 1.0);
    assertFalse(state.isConfigurationReady());
    assertEquals(ServiceAction.APPLY_UPDATE, state.beginService(1.0));

    state.desiredRevisionChanged(5, 1.1);
    state.operationSucceeded(4, 1.2);
    assertFalse(state.isConfigurationReady());
    assertEquals(ServiceAction.APPLY_UPDATE, state.peekServiceAction(1.2));

    state.beginService(1.2);
    state.operationSucceeded(5, 1.3);
    assertTrue(state.isConfigurationReady());
  }

  @Test
  void retriesUseBoundedBackoffAndRequireFullReset() {
    SparkRecoveryState state = new SparkRecoveryState(0.0, 1);

    state.operationFailed(0.5, "timeout");
    assertEquals(ServiceAction.NONE, state.peekServiceAction(0.99));
    assertEquals(ServiceAction.PROBE_AND_APPLY_RESET, state.peekServiceAction(1.0));

    state.operationFailed(1.0, "timeout");
    assertEquals(ServiceAction.NONE, state.peekServiceAction(1.99));
    assertEquals(ServiceAction.PROBE_AND_APPLY_RESET, state.peekServiceAction(2.0));

    assertEquals(0.5, SparkRecoveryState.retryDelaySeconds(1));
    assertEquals(1.0, SparkRecoveryState.retryDelaySeconds(2));
    assertEquals(2.0, SparkRecoveryState.retryDelaySeconds(3));
    assertEquals(4.0, SparkRecoveryState.retryDelaySeconds(4));
    assertEquals(5.0, SparkRecoveryState.retryDelaySeconds(99));
  }

  @Test
  void resetImmediatelyRevokesOutputAndSchedulesFullConfig() {
    SparkRecoveryState state = readyState(1);

    state.resetDetected(2.0);

    assertFalse(state.isConfigurationReady());
    assertEquals(ServiceAction.PROBE_AND_APPLY_RESET, state.peekServiceAction(2.0));
  }

  @Test
  void enabledCancellationDoesNotCountAsCommunicationFailure() {
    SparkRecoveryState state = new SparkRecoveryState(0.0, 1);
    state.beginService(0.5);

    state.operationCancelled(0.6);

    assertEquals(0, state.getConsecutiveFailures());
    assertEquals(ServiceAction.PROBE_AND_APPLY_RESET, state.peekServiceAction(0.6));
  }

  @Test
  void uncertainPersistResultIsNotBlindlyRetried() {
    SparkRecoveryState state = new SparkRecoveryState(0.0, 1);
    ServiceAction action = state.beginService(0.5);
    assertTrue(state.shouldPersist(action));

    state.persistenceAttempted();
    state.operationFailed(0.6, "persist timeout");
    ServiceAction retry = state.beginService(1.1);

    assertEquals(ServiceAction.PROBE_AND_APPLY_RESET, retry);
    assertFalse(state.shouldPersist(retry));
    assertTrue(state.getSummary().contains("PERSIST_UNCONFIRMED"));
  }

  @Test
  void reportsWhenThePersistedRevisionIsOlderThanDesiredConfiguration() {
    SparkRecoveryState state = new SparkRecoveryState(0.0, 4);
    ServiceAction action = state.beginService(0.5);
    assertTrue(state.shouldPersist(action));

    state.persistenceAttempted();
    state.desiredRevisionChanged(5, 0.55);
    state.persistenceSucceeded(4);
    state.operationSucceeded(4, 0.6);

    assertTrue(state.getSummary().contains("PERSIST_STALE"));
    assertFalse(state.isConfigurationReady());
  }

  private static SparkRecoveryState readyState(long revision) {
    SparkRecoveryState state = new SparkRecoveryState(0.0, revision);
    state.beginService(0.5);
    state.operationSucceeded(revision, 0.6);
    return state;
  }
}
