package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import java.util.Arrays;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.*;

class DriveControlTest {
    static RobotContainer robot;
    static XboxControllerSim joystick;
    static CommandScheduler scheduler;

    @BeforeAll static void init() {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        joystick = new XboxControllerSim(0);
        joystick.setAxisCount(6);
        joystick.setButtonCount(10);
        joystick.setPOVCount(1);
        joystick.setPOV(-1);
        scheduler = CommandScheduler.getInstance();
        robot = new RobotContainer();
    }

    @BeforeEach void reset() {
        scheduler.cancelAll();
        for (int i = 1; i <= 10; i++) joystick.setRawButton(i, false);
        for (int i = 0; i < 6; i++) joystick.setRawAxis(i, 0);
        mode(false, false, false);
        tick();
        SmartDashboard.putNumber(DriveSettings.MAX_SPEED_MPS, 10.24);
        SmartDashboard.putNumber(DriveSettings.MAX_ANGULAR_RATE, Math.PI * 1.5);
        SmartDashboard.putNumber(DriveSettings.AUTO_SPEED_MPS, 0.5);
        mode(true, false, false);
        tick();
        tick();
    }

    static void mode(boolean enabled, boolean auto, boolean test) {
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.setAutonomous(auto);
        DriverStationSim.setTest(test);
        DriverStationSim.notifyNewData();
    }

    static void tick() {
        DriverStationSim.notifyNewData();
        scheduler.run();
    }

    static double targetMax() throws InterruptedException {
        // CTREの別スレッドが新しいrequestを反映するまで待つ。
        Thread.sleep(100);
        return Arrays.stream(robot.drivetrain.getState().ModuleTargets)
            .mapToDouble(v -> Math.abs(v.speedMetersPerSecond)).max().orElseThrow();
    }

    static SwerveRequest.FieldCentric teleopRequest() throws Exception {
        Field f = RobotContainer.class.getDeclaredField("drive");
        f.setAccessible(true);
        return (SwerveRequest.FieldCentric) f.get(robot);
    }

    static void assertNeutralOutputs() throws Exception {
        // Allow the odometry thread and simulated CAN status signals to update.
        Thread.sleep(100);
        for (var module : robot.drivetrain.getModules()) {
            for (var motor : new com.ctre.phoenix6.hardware.TalonFX[] {
                    module.getDriveMotor(), module.getSteerMotor()}) {
                var mode = motor.getControlMode().refresh();
                assertTrue(mode.getStatus().isOK());
                assertEquals(ControlModeValue.NeutralOut, mode.getValue());
                var voltage = motor.getMotorVoltage().refresh();
                assertTrue(voltage.getStatus().isOK());
                assertEquals(0, voltage.getValueAsDouble(), 1e-6);
            }
        }
    }

    static void assertDriveOutputActive() throws Exception {
        Thread.sleep(100);
        for (var module : robot.drivetrain.getModules()) {
            var voltage = module.getDriveMotor().getMotorVoltage().refresh();
            assertTrue(voltage.getStatus().isOK());
            assertTrue(Math.abs(voltage.getValueAsDouble()) > 0.01,
                "The simulated drive output must be active before testing its removal");
        }
    }

    static Command startAuto() {
        mode(true, true, false);
        Command auto = robot.getAutonomousCommand();
        scheduler.schedule(auto);
        tick();
        tick();
        return auto;
    }

    @Test void autoActuallyDrivesForwardAtHalfMeterPerSecond() throws Exception {
        Command auto = startAuto();
        assertTrue(auto.isScheduled());
        assertEquals(0.5, targetMax(), 1e-6);
        for (var state : robot.drivetrain.getState().ModuleTargets) {
            assertEquals(0.5, state.speedMetersPerSecond * state.angle.getCos(), 1e-6);
            assertEquals(0, state.speedMetersPerSecond * state.angle.getSin(), 1e-6);
        }
    }

    @Test void driverButtonsCannotInterruptAuto() {
        Command auto = startAuto();
        joystick.setLeftBumperButton(true);
        joystick.setAButton(true);
        tick();
        assertTrue(auto.isScheduled());
        joystick.setBackButton(true);
        joystick.setYButton(true);
        tick();
        assertTrue(auto.isScheduled());
        joystick.setBackButton(false);
        joystick.setYButton(false);
        joystick.setStartButton(true);
        joystick.setXButton(true);
        tick();
        assertTrue(auto.isScheduled());
        assertSame(auto, robot.drivetrain.getCurrentCommand());
    }

    @Test void defaultDriveIgnoresJoystickInAutoAndTest() throws Exception {
        joystick.setLeftY(-0.5);
        for (boolean auto : new boolean[] {true, false}) {
            mode(true, false, false);
            tick();
            tick();
            assertDriveOutputActive();
            mode(true, auto, !auto);
            tick();
            tick();
            assertNeutralOutputs();
        }
    }

    @Test void sysIdRequiresTestMode() {
        joystick.setBackButton(true);
        joystick.setYButton(true);
        tick();
        assertSame(robot.drivetrain.getDefaultCommand(), robot.drivetrain.getCurrentCommand());
        mode(true, false, true);
        tick();
        assertNotSame(robot.drivetrain.getDefaultCommand(), robot.drivetrain.getCurrentCommand());
        assertNotNull(robot.drivetrain.getCurrentCommand());
        mode(true, false, false);
        tick();
        tick();
        assertSame(robot.drivetrain.getDefaultCommand(), robot.drivetrain.getCurrentCommand());
    }

    @Test void dashboardUpdatesTranslationRotationAndDeadbandsLive() throws Exception {
        joystick.setLeftY(-0.5);
        joystick.setRightX(-0.5);
        SmartDashboard.putNumber(DriveSettings.MAX_SPEED_MPS, 2);
        SmartDashboard.putNumber(DriveSettings.MAX_ANGULAR_RATE, 1);
        tick();
        var request = teleopRequest();
        assertEquals(1, request.VelocityX);
        assertEquals(0.5, request.RotationalRate);
        assertEquals(0.2, request.Deadband);
        assertEquals(0.1, request.RotationalDeadband);
        SmartDashboard.putNumber(DriveSettings.MAX_SPEED_MPS, 0.4);
        SmartDashboard.putNumber(DriveSettings.MAX_ANGULAR_RATE, 0.2);
        tick();
        assertEquals(0.2, request.VelocityX);
        assertEquals(0.1, request.RotationalRate);
        assertEquals(0.04, request.Deadband, 1e-9);
        assertEquals(0.02, request.RotationalDeadband, 1e-9);
        SmartDashboard.putNumber(DriveSettings.MAX_SPEED_MPS, 0);
        SmartDashboard.putNumber(DriveSettings.MAX_ANGULAR_RATE, 0);
        tick();
        assertEquals(0, targetMax(), 1e-6);
    }

    @Test void autoSpeedIsCapturedAtStartThenNeutralizesAfterFiveSeconds() throws Exception {
        SmartDashboard.putNumber(DriveSettings.AUTO_SPEED_MPS, 0.7);
        Command auto = startAuto();
        SmartDashboard.putNumber(DriveSettings.AUTO_SPEED_MPS, 1.2);
        tick();
        assertEquals(0.7, targetMax(), 1e-6);
        assertDriveOutputActive();
        SimHooks.stepTimingAsync(5.1);
        tick();
        tick();
        assertTrue(auto.isScheduled());
        assertNeutralOutputs();
    }

    @Test void cancellingAutoNeutralizesWithoutAnotherSchedulerTick() throws Exception {
        Command auto = startAuto();
        assertEquals(0.5, targetMax(), 1e-6);
        assertDriveOutputActive();
        scheduler.cancel(auto);
        assertNeutralOutputs();
    }

    @Test void disablingAutoNeutralizesAndTeleopCanResume() throws Exception {
        Command auto = startAuto();
        assertDriveOutputActive();
        mode(false, true, false);
        tick();
        assertFalse(auto.isScheduled());
        assertNeutralOutputs();
        joystick.setLeftY(-0.5);
        mode(true, false, false);
        tick();
        tick();
        assertDriveOutputActive();
    }

    @Test void neutralRequestCanBeReusedAcrossAutoRuns() throws Exception {
        for (int run = 0; run < 2; run++) {
            Command auto = startAuto();
            assertDriveOutputActive();
            scheduler.cancel(auto);
            assertNeutralOutputs();
        }
    }

    @Test void settingsAreFiniteBoundedPersistentAndPreserveExistingValues() {
        var settings = new DriveSettings(10.24, Math.PI * 1.5);
        SmartDashboard.putNumber(DriveSettings.MAX_SPEED_MPS, 100);
        SmartDashboard.putNumber(DriveSettings.MAX_ANGULAR_RATE, -1);
        SmartDashboard.putNumber(DriveSettings.AUTO_SPEED_MPS, 10);
        assertEquals(10.24, settings.maxSpeedMps());
        assertEquals(0, settings.maxAngularRateRadPerSec());
        assertEquals(2, settings.autoSpeedMps());
        SmartDashboard.putNumber(DriveSettings.MAX_SPEED_MPS, Double.NaN);
        SmartDashboard.putNumber(DriveSettings.MAX_ANGULAR_RATE, Double.NEGATIVE_INFINITY);
        SmartDashboard.putNumber(DriveSettings.AUTO_SPEED_MPS, Double.POSITIVE_INFINITY);
        assertEquals(10.24, settings.maxSpeedMps());
        assertEquals(Math.PI * 1.5, settings.maxAngularRateRadPerSec());
        assertEquals(0.5, settings.autoSpeedMps());
        SmartDashboard.putNumber(DriveSettings.MAX_SPEED_MPS, 3);
        assertEquals(3, new DriveSettings(10.24, Math.PI * 1.5).maxSpeedMps());
        for (String key : new String[] {DriveSettings.MAX_SPEED_MPS,
                DriveSettings.MAX_ANGULAR_RATE, DriveSettings.AUTO_SPEED_MPS}) {
            assertTrue(SmartDashboard.isPersistent(key));
        }
    }

    @AfterAll static void close() throws Exception {
        scheduler.cancelAll();
        scheduler.getDefaultButtonLoop().clear();
        scheduler.unregisterAllSubsystems();
        Field notifier = robot.drivetrain.getClass().getDeclaredField("m_simNotifier");
        notifier.setAccessible(true);
        ((Notifier) notifier.get(robot.drivetrain)).close();
        robot.drivetrain.close();
        for (Field field : robot.shooter.getClass().getDeclaredFields()) {
            field.setAccessible(true);
            if (field.get(robot.shooter) instanceof AutoCloseable motor) motor.close();
        }
    }
}
