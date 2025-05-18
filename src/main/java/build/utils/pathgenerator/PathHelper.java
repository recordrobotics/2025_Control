package build.utils.pathgenerator;

import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.File;

public final class PathHelper {
  private PathHelper() {}

  public static void editLinkedWaypoint(String name, Pose2d pose) {
    for (File path :
        new File(FileUtils.getLaunchDirectory(), "src/main/deploy/pathplanner/paths").listFiles()) {

      var obj = FileUtils.readJson(path);

      for (var waypoint : obj.get("waypoints")) {

        if (waypoint.get("linkedName") != null
            && name.equals(waypoint.get("linkedName").asText())) {

          System.out.println("Editing waypoint " + name + " in " + path.getName());

          if (waypoint.has("nextControl") && waypoint.get("nextControl").isObject()) {

            if (!waypoint.has("prevControl")
                || waypoint.get("prevControl").isNull()) { // first waypoint
              ((ObjectNode) obj.get("idealStartingState"))
                  .put("rotation", pose.getRotation().getDegrees());
            }

            double controlHandleOffsetX =
                waypoint.get("nextControl").get("x").asDouble()
                    - waypoint.get("anchor").get("x").asDouble();
            double controlHandleOffsetY =
                waypoint.get("nextControl").get("y").asDouble()
                    - waypoint.get("anchor").get("y").asDouble();

            ((ObjectNode) waypoint.get("nextControl")).put("x", pose.getX() + controlHandleOffsetX);
            ((ObjectNode) waypoint.get("nextControl")).put("y", pose.getY() + controlHandleOffsetY);
          }

          if (waypoint.has("prevControl") && waypoint.get("prevControl").isObject()) {

            if (!waypoint.has("nextControl")
                || waypoint.get("nextControl").isNull()) { // first waypoint
              ((ObjectNode) obj.get("goalEndState"))
                  .put("rotation", pose.getRotation().getDegrees());
            }

            double controlHandleOffsetX =
                waypoint.get("prevControl").get("x").asDouble()
                    - waypoint.get("anchor").get("x").asDouble();
            double controlHandleOffsetY =
                waypoint.get("prevControl").get("y").asDouble()
                    - waypoint.get("anchor").get("y").asDouble();

            ((ObjectNode) waypoint.get("prevControl")).put("x", pose.getX() + controlHandleOffsetX);
            ((ObjectNode) waypoint.get("prevControl")).put("y", pose.getY() + controlHandleOffsetY);
          }

          ((ObjectNode) waypoint.get("anchor")).put("x", pose.getX());
          ((ObjectNode) waypoint.get("anchor")).put("y", pose.getY());
        }
      }

      FileUtils.writeJson(path, obj);
    }
  }
}
