package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.utils.SparkSimulationHandle.RawTelemetry;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.time.Duration;
import java.util.Arrays;
import java.util.OptionalDouble;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class SparkSimulationHandleTest {
  private static final int TEST_CAN_ID = 61;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void rawTelemetryAndFaultInjectionUseTheOfficialSimDeviceWithoutExposingMotorControl() {
    try (SparkMax motor = new SparkMax(TEST_CAN_ID, MotorType.kBrushless)) {
      SparkSimulationHandle handle = new SparkSimulationHandle(motor, new Object());
      RawTelemetry telemetry = new RawTelemetry(0.125, 4.5, -321.0, 12.75, 11.8);

      handle.injectRawTelemetry(telemetry);
      var observed = handle.observe();

      SimDeviceSim faultDevice = new SimDeviceSim(
          "SPARK MAX [" + TEST_CAN_ID + "] FAULT MANAGER");
      SimBoolean otherFault = faultDevice.getBoolean("Other Fault");
      SimBoolean canFault = faultDevice.getBoolean("CAN Fault");
      SimBoolean otherStickyFault = faultDevice.getBoolean("Other Sticky Fault");
      SimBoolean canStickyFault = faultDevice.getBoolean("CAN Sticky Fault");
      SimBoolean sensorWarning = faultDevice.getBoolean("Sensor Warning");
      SimBoolean resetWarning = faultDevice.getBoolean("Has Reset Warning");
      SimBoolean sensorStickyWarning = faultDevice.getBoolean("Sensor Sticky Warning");
      SimBoolean resetStickyWarning = faultDevice.getBoolean("Has Reset Sticky Warning");

      assertAll(
          () -> assertEquals(TEST_CAN_ID, observed.canId()),
          () -> assertEquals(telemetry.appliedOutput(), observed.appliedOutput()),
          () -> assertEquals(telemetry.motorCurrentAmps(), observed.motorCurrentAmps()),
          () -> assertEquals(telemetry.velocity(), observed.velocity()),
          () -> assertEquals(telemetry.position(), observed.position()),
          () -> assertEquals(telemetry.busVoltage(), observed.busVoltage()),
          () -> assertNotNull(otherFault),
          () -> assertNotNull(canFault),
          () -> assertNotNull(otherStickyFault),
          () -> assertNotNull(canStickyFault),
          () -> assertNotNull(sensorWarning),
          () -> assertNotNull(resetWarning),
          () -> assertNotNull(sensorStickyWarning),
          () -> assertNotNull(resetStickyWarning));

      otherFault.set(true);
      otherStickyFault.set(true);
      sensorWarning.set(true);
      sensorStickyWarning.set(true);
      handle.setCanFault(true, true);
      handle.setResetWarning(true, true);

      assertAll(
          () -> assertTrue(otherFault.get(), "CAN injection must preserve unrelated faults"),
          () -> assertTrue(canFault.get()),
          () -> assertTrue(otherStickyFault.get()),
          () -> assertTrue(canStickyFault.get()),
          () -> assertTrue(sensorWarning.get(), "reset injection must preserve unrelated warnings"),
          () -> assertTrue(resetWarning.get()),
          () -> assertTrue(sensorStickyWarning.get()),
          () -> assertTrue(resetStickyWarning.get()));

      handle.setCanFault(false, false);
      handle.setResetWarning(false, false);
      assertAll(
          () -> assertTrue(otherFault.get()),
          () -> assertFalse(canFault.get()),
          () -> assertTrue(otherStickyFault.get()),
          () -> assertFalse(canStickyFault.get()),
          () -> assertTrue(sensorWarning.get()),
          () -> assertFalse(resetWarning.get()),
          () -> assertTrue(sensorStickyWarning.get()),
          () -> assertFalse(resetStickyWarning.get()));
      handle.closeSimulationResourcesForTesting();
    }
  }

  @Test
  void facadeHasNoPublicConstructorOrPhysicsAndReferenceMutationApi() {
    assertEquals(0, SparkSimulationHandle.class.getConstructors().length);
    assertTrue(Arrays.stream(SparkSimulationHandle.class.getMethods())
        .filter(method -> Modifier.isPublic(method.getModifiers()))
        .map(Method::getName)
        .noneMatch(name -> name.equals("iterate")
            || name.equals("setSetpoint")
            || name.toLowerCase().contains("reference")));
  }

  @Test
  void containerConsumesRawFramesAndFailsClosedOnCanFaultWithoutCreatingAReference()
      throws InterruptedException {
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SparkMAXContainer.configureProcessDefaults();

    SparkMAXContainer container = new SparkMAXContainer(60);
    SparkSimulationHandle handle = SparkMAXContainer.getSimulationHandleForId(60).orElseThrow();
    try {
      container.setCurrentLimit(10.0);
      handle.injectRawTelemetry(new RawTelemetry(0.0, 0.0, 0.0, 0.0, 12.0));

      assertTrue(await(5.0, container::isReady), container::getDiagnosticStatus);
      long continuityEpoch = container.getPositionContinuityEpoch();

      handle.injectRawTelemetry(new RawTelemetry(0.0, 1.0, 15.0, 42.0, 12.0));
      assertTrue(await(2.0, () -> {
        SparkMAXContainer.serviceAll();
        OptionalDouble position = container.getPositionIfReady();
        return position.isPresent() && Math.abs(position.getAsDouble() - 42.0) < 1e-9;
      }), container::getDiagnosticStatus);
      assertAll(
          () -> assertEquals("UNREFERENCED", container.getPositionReferenceStatus(null)),
          () -> assertEquals(continuityEpoch, container.getPositionContinuityEpoch(),
              "raw simulation position must never establish or mutate a reference token"));

      assertFalse(
          container.setDutyCycleIfAuthorized(0.03, () -> false),
          "a revoked one-shot authorization must be checked inside the output boundary");
      assertEquals(0.0, handle.observe().setpoint(), 1e-9);

      handle.setCanFault(true, false);
      assertTrue(await(2.0, () -> {
        SparkMAXContainer.serviceAll();
        return !container.isReady() && container.getDiagnosticStatus().contains("can=true");
      }), container::getDiagnosticStatus);
      assertFalse(container.setDutyCycle(0.03),
          "nonzero output must stay rejected after the simulated CAN fault");
    } finally {
      handle.setCanFault(false, false);
      handle.setResetWarning(false, false);
      assertTrue(await(2.0, SparkMAXContainer::cleanupSimulationDevicesForTesting),
          "simulation registry must be quiescent and reusable after the test");
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    }
  }

  @Test
  void simulationFaultInjectionSurvivesSameCanIdReconstruction() throws InterruptedException {
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    try (SimDevice unrelatedOwner = SimDevice.create("Unrelated Lifecycle Sentinel")) {
      assertNotNull(unrelatedOwner);
      SimBoolean unrelatedValue =
          unrelatedOwner.createBoolean("Alive", Direction.kBidir, false);
      assertNotNull(unrelatedValue);

      for (int cycle = 1; cycle <= 2; cycle++) {
        SparkMAXContainer container = new SparkMAXContainer(62);
        SparkSimulationHandle handle =
            SparkMAXContainer.getSimulationHandleForId(62).orElseThrow();
        try {
          handle.setCanFault(true, false);
          assertTrue(
              (handle.observe().activeFaultBits() & (1 << 3)) != 0,
              "CAN fault injection must work after reconstruction cycle " + cycle);
        } finally {
          handle.setCanFault(false, false);
          assertTrue(
              await(2.0, SparkMAXContainer::cleanupSimulationDevicesForTesting),
              "simulation registry must be reusable after cycle " + cycle);
        }
        AtomicBoolean staleAuthorizationInvoked = new AtomicBoolean();
        assertFalse(container.setDutyCycleIfAuthorized(0.03, () -> {
          staleAuthorizationInvoked.set(true);
          return true;
        }), "a retained wrapper must reject output after its native simulation owner is closed");
        assertFalse(staleAuthorizationInvoked.get(),
            "teardown rejection must occur before an external callback can block or re-enter");
        unrelatedValue.set(cycle % 2 == 1);
        assertEquals(
            cycle % 2 == 1,
            unrelatedValue.get(),
            "SPARK cleanup must not invalidate unrelated HAL SimDevice owners");
      }
    }

    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  @Test
  void blockedExternalAuthorizationCannotDelayStopAndCannotWriteAfterStop()
      throws InterruptedException {
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SparkMAXContainer.configureProcessDefaults();
    long generation = ProcessOutputSafety.revoke("SPARK_LANE_TEST_PREPARE");
    assertTrue(ProcessOutputSafety.authorize(generation));

    SparkMAXContainer container = new SparkMAXContainer(63);
    SparkSimulationHandle handle = SparkMAXContainer.getSimulationHandleForId(63).orElseThrow();
    CountDownLatch authorizationEntered = new CountDownLatch(1);
    CountDownLatch releaseAuthorization = new CountDownLatch(1);
    AtomicBoolean requestResult = new AtomicBoolean(true);
    Thread requestThread = null;
    try {
      container.setCurrentLimit(10.0);
      handle.injectRawTelemetry(new RawTelemetry(0.0, 0.0, 0.0, 0.0, 12.0));
      assertTrue(await(5.0, container::isReady), container::getDiagnosticStatus);

      requestThread = new Thread(() -> requestResult.set(container.setDutyCycleIfAuthorized(
          0.03,
          () -> {
            authorizationEntered.countDown();
            try {
              return releaseAuthorization.await(2, TimeUnit.SECONDS);
            } catch (InterruptedException exception) {
              Thread.currentThread().interrupt();
              return false;
            }
          })), "spark-blocked-authorization-test");
      requestThread.start();
      assertTrue(authorizationEntered.await(1, TimeUnit.SECONDS));

      SparkMAXContainer.OutputStopBatch stop = assertTimeoutPreemptively(
          Duration.ofMillis(500), () -> SparkMAXContainer.requestOutputStops(63));
      assertTrue(await(2.0, () -> stop.snapshot().confirmed()), stop.snapshot()::summary);

      releaseAuthorization.countDown();
      requestThread.join(2000L);
      assertFalse(requestThread.isAlive());
      assertFalse(requestResult.get(), "the stop sequence must reject the stale authorization");
      assertEquals(0.0, handle.observe().setpoint(), 1e-9);
    } finally {
      releaseAuthorization.countDown();
      if (requestThread != null) {
        requestThread.join(2000L);
      }
      ProcessOutputSafety.resetForTesting();
      assertTrue(await(2.0, SparkMAXContainer::cleanupSimulationDevicesForTesting));
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    }
  }

  private static boolean await(double timeoutSeconds, BooleanSupplier condition)
      throws InterruptedException {
    long deadlineNanos = System.nanoTime() + (long) (timeoutSeconds * 1_000_000_000L);
    do {
      SparkMAXContainer.serviceAll();
      if (condition.getAsBoolean()) {
        return true;
      }
      Thread.sleep(10L);
    } while (System.nanoTime() < deadlineNanos);
    return condition.getAsBoolean();
  }
}
