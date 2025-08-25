package frc.robot.utils.maplesim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.ironmaple.simulation.Goal;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeReefBranch;

/**
 *
 *
 * <h2>Simulates a <strong>REEF</strong>s on the field.</h2>
 *
 * <p>This class simulates a <strong>REEF</strong>s on the field where <strong>CORAL</strong>s can
 * be scored. This class does not directly handle scoring uses an array of {@link
 * ReefscapeReefBranch} objects. However for all other purposes this class behaves and can be used
 * like a normal goal.
 */
// This is a MapleSim library class modified to support custom features, so ignore the warnings
@SuppressWarnings({"java:S2293", "java:S109", "java:S1104"})
public class ImprovedReefscapeReefSimulation implements SimulatedArena.Simulatable {
    protected final HashMap<String, ImprovedReefscapeReefBranch> branches;
    public ReefscapeAlgaeOnField algae;
    private StructArrayPublisher<Pose3d> reefPub;
    Pose3d[] branchPoses;

    /**
     *
     *
     * <h2>Creates an reef of the specified color.</h2>
     *
     * @param arena The host arena of this reef.
     * @param isBlue Wether this is the blue reef or the red one.
     */
    ImprovedReefscapeReefSimulation(ImprovedArena2025Reefscape arena, boolean isBlue) {
        branches = new HashMap<String, ImprovedReefscapeReefBranch>(48);
        branchPoses = new Pose3d[96];
        for (int tower = 0; tower < 12; tower++) {
            for (int level = 0; level < 4; level++) {
                char towerChar = (char) ('A' + tower);
                char levelChar = (char) ('1' + level);
                ImprovedReefscapeReefBranch branch = new ImprovedReefscapeReefBranch(arena, isBlue, level, tower);
                branches.put(String.valueOf(towerChar) + String.valueOf(levelChar), branch);
                Pose3d tempPose = branch.getPose();

                branchPoses[2 * (tower * 4 + level)] = tempPose;

                branchPoses[2 * (tower * 4 + level) + 1] =
                        new Pose3d(tempPose.getTranslation(), Goal.flipRotation(tempPose.getRotation()));
            }
        }
        reefPub = NetworkTableInstance.getDefault()
                .getStructArrayTopic(isBlue ? "BlueReef" : "RedReef", Pose3d.struct)
                .publish();

        reefPub.set(branchPoses);
    }

    public void draw(List<Pose3d> coralPosesToDisplay) {
        for (ImprovedReefscapeReefBranch branch : branches.values()) {
            branch.draw(coralPosesToDisplay);
        }
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        for (ImprovedReefscapeReefBranch branch : branches.values()) {
            branch.simulationSubTick(subTickNum);
        }
    }

    /**
     *
     *
     * <h2>Resets the reef to its original state.</h2>
     */
    public void clearReef() {
        for (ImprovedReefscapeReefBranch branch : branches.values()) {
            branch.clear();
        }
    }

    public ImprovedReefscapeReefBranch getBranch(String id) {
        return branches.get(id);
    }

    public Set<Map.Entry<String, ImprovedReefscapeReefBranch>> getBranches() {
        return branches.entrySet();
    }
}
