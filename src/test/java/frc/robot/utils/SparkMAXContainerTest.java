package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.DoubleSupplier;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

class SparkMAXContainerTest {
  private SparkMAXContainer container;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void createMotor() {
    container = new SparkMAXContainer(31);
  }

  @AfterEach
  void closeMotor() {
    container.motor.close();
  }

  @ParameterizedTest
  @CsvSource({
      "0, 10, 0.5, false",
      "20, 10, 0.5, false",
      "9.49, 10, 0.5, false",
      "10.51, 10, 0.5, false",
      "9.5, 10, 0.5, true",
      "10.5, 10, 0.5, true",
      "10, 10, 0.5, true",
      "-10, -10, 0.5, true",
      "-12, -10, 0.5, false"
  })
  void reportsArrivalOnlyWithinTolerance(double measured, double target,
      double tolerance, boolean expected) {
    useEncoder(() -> measured);

    assertEquals(expected, container.goToPostion(target, tolerance));
    assertEquals(target, container.exposeReference().getSetpoint(), 1e-5);
  }

  @Test
  void defaultToleranceIsHalfARotation() {
    useEncoder(() -> 10.5);
    assertTrue(container.goToPostion(10));
    useEncoder(() -> 10.51);
    assertFalse(container.goToPostion(10));
  }

  @Test
  void missingEncoderDoesNotReportArrival() {
    container.encoder = null;
    assertFalse(container.goToPostion(0));
  }

  @Test
  void encoderExceptionWithoutAMessageDoesNotReportArrival() {
    useEncoder(() -> { throw new IllegalStateException(); });
    assertFalse(container.goToPostion(0));
  }

  @Test
  void closedControllerDoesNotReportArrival() {
    useEncoder(() -> 0);
    container.motor.close();
    assertFalse(container.goToPostion(0));
  }

  @Test
  void encoderReadErrorDoesNotReportArrivalOrSendASetpoint() {
    container.motor.close();
    container.motor = new SparkMax(31, MotorType.kBrushless) {
      @Override
      public REVLibError getLastError() {
        return REVLibError.kTimeout;
      }
    };
    useEncoder(() -> 10);
    double previousSetpoint = container.exposeReference().getSetpoint();

    assertFalse(container.goToPostion(10));
    assertEquals(previousSetpoint, container.exposeReference().getSetpoint());
  }

  @ParameterizedTest
  @ValueSource(doubles = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
  void nonFiniteMeasurementDoesNotReportArrival(double measured) {
    useEncoder(() -> measured);
    assertFalse(container.goToPostion(0));
  }

  @ParameterizedTest
  @CsvSource({"NaN, 0.5", "Infinity, 0.5", "-Infinity, 0.5",
      "10, NaN", "10, Infinity", "10, -1", "10, 0"})
  void invalidReferenceDoesNotSendASetpoint(double target, double tolerance) {
    useEncoder(() -> 10);
    double previousSetpoint = container.exposeReference().getSetpoint();

    assertFalse(container.goToPostion(target, tolerance));
    assertEquals(previousSetpoint, container.exposeReference().getSetpoint());
  }

  private void useEncoder(DoubleSupplier position) {
    container.encoder = new RelativeEncoder() {
      @Override
      public double getPosition() {
        return position.getAsDouble();
      }

      @Override
      public double getVelocity() {
        throw new UnsupportedOperationException();
      }

      @Override
      public REVLibError setPosition(double value) {
        throw new UnsupportedOperationException();
      }
    };
  }
}
