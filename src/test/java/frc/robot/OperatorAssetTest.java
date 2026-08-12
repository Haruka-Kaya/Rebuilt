package frc.robot;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.commands.HardwareSelfTestCommand;
import frc.robot.constants.ConfiguredCanHardware;
import frc.robot.constants.ConfiguredOperatorControls;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.Test;

class OperatorAssetTest {
  private static final Path COMP_LAYOUT = Path.of("elastic_configs", "comp.json");
  private static final Path SIM_DRIVER_STATION = Path.of("simgui-ds.json");
  private static final int DRIVER_PORT = 0;
  private static final int OPERATOR_PORT = 1;
  private static final int MAINTENANCE_PORT = 2;
  // RobotContainer binds driver raw button 14 and maintenance deadman raw button 10.
  private static final int REQUIRED_DRIVER_BUTTONS =
      ConfiguredOperatorControls.maximumDriverButton();
  private static final int REQUIRED_DRIVER_AXES = 3;
  private static final int REQUIRED_OPERATOR_BUTTONS =
      ConfiguredOperatorControls.maximumOperatorButton();
  private static final int REQUIRED_MAINTENANCE_BUTTONS =
      ConfiguredOperatorControls.maximumMaintenanceButton();

  private static final Map<String, WidgetExpectation> REQUIRED_DIAGNOSTIC_WIDGETS = Map.ofEntries(
      Map.entry("/SmartDashboard/Auto Chooser", new WidgetExpectation("ComboBox Chooser", null)),
      Map.entry(
          "/SmartDashboard/Autonomous/Ready", new WidgetExpectation("Boolean Box", "boolean")),
      Map.entry(
          "/SmartDashboard/Autonomous/Status",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Autonomous/Selected",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Autonomous/Last Result",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Runtime/Scheduler Healthy",
          new WidgetExpectation("Boolean Box", "boolean")),
      Map.entry(
          "/SmartDashboard/Runtime/Fault",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Hardware Self-Test/Armed",
          new WidgetExpectation("Toggle Button", "boolean")),
      Map.entry(
          "/SmartDashboard/Hardware Self-Test/Arm Valid",
          new WidgetExpectation("Boolean Box", "boolean")),
      Map.entry(
          "/SmartDashboard/Hardware Self-Test/Running",
          new WidgetExpectation("Boolean Box", "boolean")),
      Map.entry(
          "/SmartDashboard/Hardware Self-Test/Overall",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Hardware Self-Test/Results",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Hardware Self-Test/Run ID",
          new WidgetExpectation("Text Display", "double")),
      Map.entry(
          "/SmartDashboard/Hardware Self-Test/Coverage",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Hardware Self-Test/CAN Results",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Hardware Self-Test/Abort Reason",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Hardware/CAN Configured",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Hardware/SPARK Health",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Hardware/CTRE Health",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Hardware Self-Test/GLOBAL_START/Stop Result",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Hardware Self-Test/GLOBAL_END/Stop Result",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Controls/Configured",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Climber Diagnostic/Armed",
          new WidgetExpectation("Toggle Button", "boolean")),
      Map.entry(
          "/SmartDashboard/Climber Diagnostic/Brushless Motor Type Verified",
          new WidgetExpectation("Toggle Button", "boolean")),
      Map.entry(
          "/SmartDashboard/Unhomed Actuator Diagnostic/Armed",
          new WidgetExpectation("Toggle Button", "boolean")),
      Map.entry(
          "/SmartDashboard/Unhomed Actuator Diagnostic/Arm Valid",
          new WidgetExpectation("Boolean Box", "boolean")),
      Map.entry(
          "/SmartDashboard/Unhomed Actuator Diagnostic/Physical Clearance Verified",
          new WidgetExpectation("Toggle Button", "boolean")),
      Map.entry(
          "/SmartDashboard/Unhomed Actuator Diagnostic/Brushless Motor Type Verified",
          new WidgetExpectation("Toggle Button", "boolean")),
      Map.entry(
          "/SmartDashboard/Unhomed Actuator Diagnostic/Target ID30 Intake",
          new WidgetExpectation("Toggle Button", "boolean")),
      Map.entry(
          "/SmartDashboard/Unhomed Actuator Diagnostic/Target ID38 Shooter",
          new WidgetExpectation("Toggle Button", "boolean")),
      Map.entry(
          "/SmartDashboard/Unhomed Actuator Diagnostic/Target ID39 Turret",
          new WidgetExpectation("Toggle Button", "boolean")),
      Map.entry(
          "/SmartDashboard/Unhomed Actuator Diagnostic/Direction Negative",
          new WidgetExpectation("Toggle Button", "boolean")),
      Map.entry(
          "/SmartDashboard/Unhomed Actuator Diagnostic/Direction Positive",
          new WidgetExpectation("Toggle Button", "boolean")),
      Map.entry(
          "/SmartDashboard/Unhomed Actuator Diagnostic/Status",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Unhomed Actuator Diagnostic/Stop Evidence",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Climber/Controllers Ready",
          new WidgetExpectation("Boolean Box", "boolean")),
      Map.entry(
          "/SmartDashboard/Climber/Status",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Feeder/Known Fault Motion Blocked",
          new WidgetExpectation("Boolean Box", "boolean")),
      Map.entry(
          "/SmartDashboard/Intake Actuator/Reference State",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Intake Actuator/Position Valid",
          new WidgetExpectation("Boolean Box", "boolean")),
      Map.entry(
          "/SmartDashboard/Shooter Actuator/Reference State",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Shooter Actuator/Position Valid",
          new WidgetExpectation("Boolean Box", "boolean")),
      Map.entry(
          "/SmartDashboard/Turret/Reference State",
          new WidgetExpectation("Large Text Display", "string")),
      Map.entry(
          "/SmartDashboard/Turret/Position Valid",
          new WidgetExpectation("Boolean Box", "boolean")));

  private final ObjectMapper objectMapper = new ObjectMapper();

  @Test
  void competitionLayoutExposesRequiredDiagnosticsAndSafetyControls() throws IOException {
    JsonNode root = objectMapper.readTree(COMP_LAYOUT.toFile());
    JsonNode diagnosticsTab = findTab(root, "Diagnostics / Setup");
    assertNotNull(diagnosticsTab, "Diagnostics / Setup tab is missing");

    Map<String, JsonNode> widgetsByTopic = new HashMap<>();
    diagnosticsTab.at("/grid_layout/containers").forEach(widget -> {
      String topic = widget.at("/properties/topic").asText();
      assertTrue(widgetsByTopic.put(topic, widget) == null, () -> "Duplicate widget topic: " + topic);
    });

    REQUIRED_DIAGNOSTIC_WIDGETS.forEach((topic, expectation) -> {
      JsonNode widget = widgetsByTopic.get(topic);
      assertNotNull(widget, () -> "Required diagnostics widget is missing: " + topic);
      assertEquals(expectation.type(), widget.path("type").asText(), topic);
      if (expectation.dataType() != null) {
        assertEquals(expectation.dataType(), widget.at("/properties/data_type").asText(), topic);
      }
    });
  }

  @Test
  void diagnosticsWidgetsHavePositiveNonoverlappingRectangles() throws IOException {
    JsonNode root = objectMapper.readTree(COMP_LAYOUT.toFile());
    JsonNode diagnosticsTab = findTab(root, "Diagnostics / Setup");
    assertNotNull(diagnosticsTab, "Diagnostics / Setup tab is missing");
    JsonNode widgets = diagnosticsTab.at("/grid_layout/containers");

    for (int first = 0; first < widgets.size(); first++) {
      JsonNode firstWidget = widgets.get(first);
      assertPositiveRectangle(firstWidget);
      for (int second = first + 1; second < widgets.size(); second++) {
        JsonNode secondWidget = widgets.get(second);
        assertFalse(
            rectanglesOverlap(firstWidget, secondWidget),
            () -> firstWidget.path("title").asText()
                + " overlaps " + secondWidget.path("title").asText());
      }
    }
  }

  @Test
  void bothAllianceLayoutsExposeTheHubReleaseInterlock() throws IOException {
    JsonNode root = objectMapper.readTree(COMP_LAYOUT.toFile());

    assertAll(
        () -> assertTabContainsTopic(root, "Red Alliance", "/SmartDashboard/Hub Active"),
        () -> assertTabContainsTopic(root, "Blue Alliance", "/SmartDashboard/Hub Active"));
  }

  @Test
  void simulatedControllersCoverEveryProductionButtonBinding() throws IOException {
    JsonNode root = objectMapper.readTree(SIM_DRIVER_STATION.toFile());
    JsonNode joysticks = root.path("keyboardJoysticks");
    JsonNode robotJoysticks = root.path("robotJoysticks");

    assertAll(
        () -> assertEquals(6, robotJoysticks.size(), "HAL robot joystick assignment count"),
        () -> assertRobotControllerAssignment(robotJoysticks, DRIVER_PORT, "Keyboard0"),
        () -> assertRobotControllerAssignment(robotJoysticks, OPERATOR_PORT, "Keyboard1"),
        () -> assertRobotControllerAssignment(robotJoysticks, MAINTENANCE_PORT, "Keyboard2"),
        () -> assertControllerMapping(
            joysticks.get(DRIVER_PORT), REQUIRED_DRIVER_BUTTONS, REQUIRED_DRIVER_AXES,
            "driver port 0"),
        () -> assertControllerMapping(
            joysticks.get(OPERATOR_PORT), REQUIRED_OPERATOR_BUTTONS, 0,
            "operator port 1"),
        () -> assertControllerMapping(
            joysticks.get(MAINTENANCE_PORT), REQUIRED_MAINTENANCE_BUTTONS, 0,
            "maintenance port 2"));
  }

  @Test
  void hardwareSelfTestCoverageUsesTheExactCtreDeviceSet() {
    String coverage = HardwareSelfTestCommand.getCoverageManifest();
    String expectedCoverage = ConfiguredCanHardware.ctreCoverageLabel()
        + " swerve=LOW_OUTPUT_MOTION_OBSERVED_ONLY";

    assertAll(
        () -> assertTrue(coverage.contains(expectedCoverage),
            "Coverage must list only the configured Pigeon, CANcoder, steer, and drive IDs"),
        () -> assertTrue(coverage.contains("Spark30 intake actuator=MANUAL_ARMED_PULSE_ONLY")),
        () -> assertTrue(coverage.contains("Spark38 shooter actuator=MANUAL_ARMED_PULSE_ONLY")),
        () -> assertTrue(coverage.contains("Spark39 turret=MANUAL_ARMED_PULSE_ONLY")),
        () -> assertFalse(coverage.contains("CTRE20/40-57"),
            "Coverage must not imply nonexistent CTRE IDs 44-49"));
  }

  @Test
  void hardwareSelfTestResetsEveryConfiguredStopBarrierResult() {
    assertEquals(
        Set.of(
            "GLOBAL_START",
            "SPARK_ID31_INTAKE_ROLLER_SPARK_STOP",
            "SPARK_ID32_FEEDER_CONTROLLED_RETEST_SPARK_STOP",
            "SPARK_ID33_CONVEYOR_SPARK_STOP",
            "SPARK_ID36_37_FLYWHEEL_PAIR_SPARK_STOP",
            "SPARK_ID37_FOLLOWER_ISOLATED_SPARK_STOP",
            "SWERVE_FORWARD_SWERVE_STOP",
            "SWERVE_STRAFE_SWERVE_STOP",
            "SWERVE_ROTATE_SWERVE_STOP",
            "GLOBAL_END"),
        Set.copyOf(HardwareSelfTestCommand.getStopResultNames()));
  }

  private static JsonNode findTab(JsonNode root, String name) {
    for (JsonNode tab : root.path("tabs")) {
      if (name.equals(tab.path("name").asText())) {
        return tab;
      }
    }
    return null;
  }

  private static void assertTabContainsTopic(JsonNode root, String tabName, String topic) {
    JsonNode tab = findTab(root, tabName);
    assertNotNull(tab, tabName + " tab is missing");
    boolean found = false;
    for (JsonNode widget : tab.at("/grid_layout/containers")) {
      found |= topic.equals(widget.at("/properties/topic").asText());
    }
    assertTrue(found, () -> tabName + " is missing " + topic);
  }

  private static void assertControllerMapping(
      JsonNode controller, int requiredButtons, int requiredAxes, String description) {
    assertNotNull(controller, description + " is missing");
    int buttonCount = controller.path("buttonCount").asInt();
    JsonNode buttonKeys = controller.path("buttonKeys");
    Set<Integer> uniqueKeys = new HashSet<>();
    buttonKeys.forEach(key -> uniqueKeys.add(key.asInt()));

    assertAll(
        () -> assertTrue(buttonCount >= requiredButtons, description + " buttonCount is too small"),
        () -> assertTrue(
            controller.path("axisCount").asInt() >= requiredAxes,
            description + " axisCount is too small"),
        () -> assertEquals(buttonCount, buttonKeys.size(), description + " buttonKeys length"),
        () -> assertEquals(buttonCount, uniqueKeys.size(), description + " button keys must be distinct"),
        () -> buttonKeys.forEach(key -> assertTrue(
            isValidGlfwKey(key.asInt()), description + " contains invalid GLFW key " + key.asInt())));
  }

  private static void assertRobotControllerAssignment(
      JsonNode robotJoysticks, int port, String expectedGuid) {
    JsonNode assignment = robotJoysticks.get(port);
    assertNotNull(assignment, "robot joystick port " + port + " is missing");
    assertAll(
        () -> assertEquals(expectedGuid, assignment.path("guid").asText(),
            "robot joystick port " + port + " GUID"),
        () -> assertTrue(assignment.has("useGamepad"),
            "robot joystick port " + port + " must persist useGamepad"),
        () -> assertFalse(assignment.path("useGamepad").asBoolean(),
            "keyboard joystick port " + port + " must not use gamepad mapping"));
  }

  private static boolean isValidGlfwKey(int key) {
    return (key >= 32 && key <= 96)
        || (key >= 161 && key <= 162)
        || (key >= 256 && key <= 269)
        || (key >= 280 && key <= 284)
        || (key >= 290 && key <= 314)
        || (key >= 320 && key <= 336)
        || (key >= 340 && key <= 348);
  }

  private static void assertPositiveRectangle(JsonNode widget) {
    assertTrue(widget.path("width").asDouble() > 0.0, widget.path("title").asText());
    assertTrue(widget.path("height").asDouble() > 0.0, widget.path("title").asText());
  }

  private static boolean rectanglesOverlap(JsonNode first, JsonNode second) {
    double firstX = first.path("x").asDouble();
    double firstY = first.path("y").asDouble();
    double secondX = second.path("x").asDouble();
    double secondY = second.path("y").asDouble();
    return firstX < secondX + second.path("width").asDouble()
        && firstX + first.path("width").asDouble() > secondX
        && firstY < secondY + second.path("height").asDouble()
        && firstY + first.path("height").asDouble() > secondY;
  }

  private record WidgetExpectation(String type, String dataType) {}
}
