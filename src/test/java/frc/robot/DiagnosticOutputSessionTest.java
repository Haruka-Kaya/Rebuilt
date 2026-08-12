package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class DiagnosticOutputSessionTest {
  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void permitIsBoundToExactDutyDeadlineAndGeneration() {
    AtomicReference<Double> clock = new AtomicReference<>(10.0);
    AtomicReference<String> tripReason = new AtomicReference<>();
    try (DiagnosticOutputSession session = new DiagnosticOutputSession(
        20.0, () -> true, tripReason::set, clock::get)) {
      var first = session.beginPulse(0.03, 0.40).orElseThrow();

      assertTrue(first.isValidFor(0.03));
      assertFalse(first.isValidFor(-0.03));
      assertFalse(first.isValidFor(Math.nextUp(0.03)));

      first.revoke();
      assertFalse(first.isValidFor(0.03));
      var second = session.beginPulse(-0.03, 0.40).orElseThrow();
      assertTrue(second.isValidFor(-0.03));

      clock.set(10.41);
      assertFalse(second.isValidFor(-0.03));
      assertEquals(null, tripReason.get(),
          "pure boundary checks do not synchronously invoke the independent alarm callback");
    }
  }

  @Test
  void clockRollbackFailsClosedAtBothSessionAndPulseBoundaries() {
    AtomicReference<Double> clock = new AtomicReference<>(100.0);
    try (DiagnosticOutputSession session = new DiagnosticOutputSession(
        110.0, () -> true, ignored -> {}, clock::get)) {
      var permit = session.beginPulse(0.03, 0.40).orElseThrow();

      clock.set(99.99);
      assertFalse(permit.isValidFor(0.03));
      permit.revoke();
      assertTrue(session.beginPulse(0.03, 0.40).isEmpty(),
          "a rolled-back clock cannot mint a fresh output authority");
    }
  }

  @Test
  void invalidInterlockAndUnboundedPulseCannotMintAuthority() {
    AtomicReference<Double> clock = new AtomicReference<>(5.0);
    try (DiagnosticOutputSession session = new DiagnosticOutputSession(
        10.0, () -> false, ignored -> {}, clock::get)) {
      assertTrue(session.beginPulse(0.03, 0.40).isEmpty());
    }
    try (DiagnosticOutputSession session = new DiagnosticOutputSession(
        10.0, () -> true, ignored -> {}, clock::get)) {
      assertTrue(session.beginPulse(0.03, Double.POSITIVE_INFINITY).isEmpty());
      assertTrue(session.beginPulse(0.03, 1.51).isEmpty());
      assertTrue(session.beginPulse(0.0, 0.40).isEmpty());
    }
  }

  @Test
  void sessionDeadlineCoversTheBoundedHstButRejectsAnythingLonger() {
    AtomicReference<Double> clock = new AtomicReference<>(10.0);
    try (DiagnosticOutputSession session = new DiagnosticOutputSession(
        10.0 + frc.robot.constants.Constants.HardwareTestConstants
            .HARDWARE_SELF_TEST_SESSION_LIFETIME_SECONDS,
        () -> true,
        ignored -> {},
        clock::get)) {
      assertTrue(session.isValid());
    }
    assertThrows(
        IllegalArgumentException.class,
        () -> new DiagnosticOutputSession(
            10.0 + frc.robot.constants.Constants.HardwareTestConstants
                .HARDWARE_SELF_TEST_SESSION_LIFETIME_SECONDS + 0.001,
            () -> true,
            ignored -> {},
            clock::get));
  }

  @Test
  void independentAlarmTripsEvenWithoutSchedulerProgress() throws InterruptedException {
    CountDownLatch tripped = new CountDownLatch(1);
    AtomicReference<String> tripReason = new AtomicReference<>();
    double now = Timer.getFPGATimestamp();
    try (DiagnosticOutputSession session = new DiagnosticOutputSession(
        now + 1.0,
        () -> true,
        reason -> {
          tripReason.set(reason);
          tripped.countDown();
        })) {
      var permit = session.beginPulse(0.03, 0.05).orElseThrow();

      assertTrue(tripped.await(1, TimeUnit.SECONDS));
      assertEquals("DIAGNOSTIC_PULSE_DEADLINE_EXPIRED", tripReason.get());
      assertFalse(permit.isValidFor(0.03));
    }
  }

  @Test
  void explicitUnconfirmedStopTripRevokesAndEscalatesExactlyOnce() {
    AtomicReference<Double> clock = new AtomicReference<>(20.0);
    AtomicReference<String> tripReason = new AtomicReference<>();
    try (DiagnosticOutputSession session = new DiagnosticOutputSession(
        25.0, () -> true, tripReason::set, clock::get)) {
      var permit = session.beginPulse(0.03, 0.40).orElseThrow();

      session.trip("STOP_EVIDENCE_TIMEOUT");
      session.trip("MUST_NOT_OVERWRITE_FIRST_TRIP");

      assertFalse(permit.isValidFor(0.03));
      assertEquals("STOP_EVIDENCE_TIMEOUT", tripReason.get());
    }
  }
}
