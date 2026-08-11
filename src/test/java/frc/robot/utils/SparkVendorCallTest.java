package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.Test;

import com.revrobotics.REVLibError;

class SparkVendorCallTest {
  @Test
  void acceptsSuccessfulCall() {
    SparkVendorCall.Result result = SparkVendorCall.execute("setpoint", () -> REVLibError.kOk);

    assertTrue(result.succeeded());
    assertEquals(REVLibError.kOk, result.error());
    assertNull(result.failure());
  }

  @Test
  void convertsRevErrorToFailure() {
    SparkVendorCall.Result result = SparkVendorCall.execute("setpoint", () -> REVLibError.kError);

    assertFalse(result.succeeded());
    assertEquals(REVLibError.kError, result.error());
    assertEquals("setpoint kError", result.failure());
  }

  @Test
  void convertsNullToFailure() {
    SparkVendorCall.Result result = SparkVendorCall.execute("follower pause", () -> null);

    assertFalse(result.succeeded());
    assertEquals(REVLibError.kError, result.error());
    assertEquals("follower pause returned null", result.failure());
  }

  @Test
  void containsRuntimeExceptionAndCallsVendorOnce() {
    AtomicInteger calls = new AtomicInteger();

    SparkVendorCall.Result result = SparkVendorCall.execute("setpoint", () -> {
      calls.incrementAndGet();
      throw new IllegalStateException("simulated JNI failure");
    });

    assertEquals(1, calls.get());
    assertFalse(result.succeeded());
    assertEquals(REVLibError.kError, result.error());
    assertEquals("setpoint exception IllegalStateException", result.failure());
  }
}
