package frc.robot;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;
import frc.robot.constants.Constants.AutoConstants;
import org.junit.jupiter.api.Test;

class AutoDeployAssetTest {
  private static final Path BUILD_GRADLE = Path.of("build.gradle");
  private static final Path PATHPLANNER_DIRECTORY =
      Path.of("src", "main", "deploy", "pathplanner");
  private static final double RUNTIME_MAX_TRANSLATION_METERS_PER_SECOND = 0.512;
  private static final double RUNTIME_MAX_ANGULAR_DEGREES_PER_SECOND = 18.0;
  private static final Set<String> REGISTERED_UNBOUNDED_NAMED_COMMANDS =
      Set.of("Advanced Fire", "Slurp");

  private final ObjectMapper objectMapper = new ObjectMapper();

  @Test
  void deployRemovesStaleRobotFilesSoDeletedAutosCannotReappear() throws IOException {
    String buildScript = Files.readString(BUILD_GRADLE);
    assertTrue(
        buildScript.contains("deleteOldFiles = true"),
        "roboRIO deploy must remove stale .auto/.path files");
  }

  @Test
  void autonomousRemainsFailClosedWhilePhysicalConfigurationIsUnverified() {
    assertFalse(AutoConstants.CALIBRATED_AUTONOMOUS_ENABLED);
  }

  @Test
  void exampleAutoKeepsAdvancedFireInsidePathDeadline() throws IOException {
    JsonNode auto = readJson(PATHPLANNER_DIRECTORY.resolve("autos/Example Auto.auto"));
    assertEquals("sequential", auto.at("/command/type").asText());

    JsonNode topLevelCommands = auto.at("/command/data/commands");
    assertTrue(topLevelCommands.isArray() && !topLevelCommands.isEmpty());
    JsonNode deadline = topLevelCommands.get(0);
    assertEquals("deadline", deadline.path("type").asText());

    Set<String> referencedPaths = new HashSet<>();
    Set<String> namedCommands = new HashSet<>();
    collectReferences(deadline, referencedPaths, namedCommands);

    assertAll(
        () -> assertEquals(Set.of("Example Path"), referencedPaths),
        () -> assertTrue(namedCommands.contains("Advanced Fire")),
        () -> assertTrue(auto.path("resetOdom").asBoolean()));

    for (String pathName : referencedPaths) {
      assertTrue(
          Files.isRegularFile(PATHPLANNER_DIRECTORY.resolve("paths/" + pathName + ".path")),
          () -> "Referenced PathPlanner path is missing: " + pathName);
    }
  }

  @Test
  void everyDeployedAutoHasABoundedCompletionPathAndValidReferences() throws IOException {
    Path autosDirectory = PATHPLANNER_DIRECTORY.resolve("autos");
    List<Path> autos;
    try (Stream<Path> files = Files.list(autosDirectory)) {
      autos = files
          .filter(path -> path.getFileName().toString().endsWith(".auto"))
          .sorted()
          .toList();
    }
    assertFalse(autos.isEmpty(), "At least one reviewed autonomous routine must be deployed");

    for (Path autoPath : autos) {
      JsonNode command = readJson(autoPath).path("command");
      Set<String> referencedPaths = new HashSet<>();
      Set<String> namedCommands = new HashSet<>();
      collectReferences(command, referencedPaths, namedCommands);

      assertTrue(
          REGISTERED_UNBOUNDED_NAMED_COMMANDS.containsAll(namedCommands),
          () -> autoPath + " references an unreviewed NamedCommand: " + namedCommands);
      assertTrue(
          isGuaranteedToFinish(command),
          () -> autoPath + " can run forever because an unbounded NamedCommand is not bounded");
      for (String pathName : referencedPaths) {
        assertTrue(
            Files.isRegularFile(PATHPLANNER_DIRECTORY.resolve("paths/" + pathName + ".path")),
            () -> autoPath + " references missing path " + pathName);
      }
    }
  }

  @Test
  void examplePathConstraintsStayWithinRuntimeOutputClamp() throws IOException {
    JsonNode path = readJson(PATHPLANNER_DIRECTORY.resolve("paths/Example Path.path"));
    JsonNode constraints = path.path("globalConstraints");

    assertAll(
        () -> assertFalse(path.path("useDefaultConstraints").asBoolean(true)),
        () -> assertTrue(
            constraints.path("maxVelocity").asDouble()
                <= RUNTIME_MAX_TRANSLATION_METERS_PER_SECOND),
        () -> assertEquals(0.5, constraints.path("maxAcceleration").asDouble()),
        () -> assertTrue(
            constraints.path("maxAngularVelocity").asDouble()
                <= RUNTIME_MAX_ANGULAR_DEGREES_PER_SECOND),
        () -> assertEquals(30.0, constraints.path("maxAngularAcceleration").asDouble()),
        () -> assertEquals(0.0, path.at("/goalEndState/velocity").asDouble()));
  }

  private JsonNode readJson(Path path) throws IOException {
    return objectMapper.readTree(path.toFile());
  }

  private static void collectReferences(
      JsonNode command, Set<String> referencedPaths, Set<String> namedCommands) {
    switch (command.path("type").asText()) {
      case "path" -> referencedPaths.add(command.at("/data/pathName").asText());
      case "named" -> namedCommands.add(command.at("/data/name").asText());
      default -> {
        JsonNode children = command.at("/data/commands");
        if (children.isArray()) {
          children.forEach(child -> collectReferences(child, referencedPaths, namedCommands));
        }
      }
    }
  }

  private static boolean isGuaranteedToFinish(JsonNode command) {
    String type = command.path("type").asText();
    JsonNode children = command.at("/data/commands");
    return switch (type) {
      case "path", "wait", "none" -> true;
      case "named" -> false;
      case "sequential", "parallel" ->
          children.isArray()
              && !children.isEmpty()
              && stream(children).allMatch(AutoDeployAssetTest::isGuaranteedToFinish);
      case "deadline" ->
          children.isArray()
              && !children.isEmpty()
              && isGuaranteedToFinish(children.get(0));
      case "race" ->
          children.isArray()
              && stream(children).anyMatch(AutoDeployAssetTest::isGuaranteedToFinish);
      default -> false;
    };
  }

  private static Stream<JsonNode> stream(JsonNode array) {
    return java.util.stream.StreamSupport.stream(array.spliterator(), false);
  }
}
