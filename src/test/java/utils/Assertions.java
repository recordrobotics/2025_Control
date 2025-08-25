package utils;

import static org.junit.jupiter.api.AssertionFailureBuilder.*;

import frc.robot.utils.maplesim.ImprovedArena2025Reefscape;
import frc.robot.utils.maplesim.ImprovedReefscapeReefBranch;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import org.ironmaple.simulation.SimulatedArena;

public class Assertions {

    /**
     * Asserts that the reef matches the expected string format.
     * The expected format is a comma-separated list of corals, where each coral
     * is represented by its branch ID (e.g., "BA4", "RC3") followed by an optional
     * count (e.g., "BA40" - blue A4 has 0 coral, "RB12" - red B1 has 2 coral).
     * Full reefstring example: "BG4,BH4,RA12" - blue G4 has 1 coral, blue H4 has 1 coral, red A1 has 2 coral.
     * @implNote Only asserts that the reef contains the specified corals, but does not check if all other branches are empty.
     * @param reefString the expected reef string
     * @throws IllegalArgumentException if the reefString is not in the expected format or contains invalid coral IDs
     */
    public static void assertReefHas(String reefString) {
        assertReefEquals(reefString, false);
    }

    /**
     * Asserts that the reef matches the expected string format.
     * The expected format is a comma-separated list of corals, where each coral
     * is represented by its branch ID (e.g., "BA4", "RC3") followed by an optional
     * count (e.g., "BA40" - blue A4 has 0 coral, "RB12" - red B1 has 2 coral).
     * Full reefstring example: "BG4,BH4,RA12" - blue G4 has 1 coral, blue H4 has 1 coral, red A1 has 2 coral.
     * @implNote Also asserts that all other branches not mentioned in the reefString are empty.
     * @param reefString the expected reef string
     * @throws IllegalArgumentException if the reefString is not in the expected format or contains invalid coral IDs
     */
    public static void assertReefEquals(String reefString) {
        assertReefEquals(reefString, true);
    }

    /**
     * Asserts that the reef matches the expected string format.
     * The expected format is a comma-separated list of corals, where each coral
     * is represented by its branch ID (e.g., "BA4", "RC3") followed by an optional
     * count (e.g., "BA40" - blue A4 has 0 coral, "RB12" - red B1 has 2 coral).
     * Full reefstring example: "BG4,BH4,RA12" - blue G4 has 1 coral, blue H4 has 1 coral, red A1 has 2 coral.
     * @param reefString the expected reef string
     * @param assertRestAreEmpty if true, also asserts that all other branches not mentioned in the reefString are empty
     * @throws IllegalArgumentException if the reefString is not in the expected format or contains invalid coral IDs
     */
    public static void assertReefEquals(String reefString, boolean assertRestAreEmpty) {
        ArrayList<String> corals = new ArrayList<>(Arrays.asList(reefString.split(",")));
        ArrayList<String> coralIds = new ArrayList<>(
                corals.stream().map(coral -> coral.trim().substring(0, 3)).toList());

        // also assert rest of the branches are empty
        if (assertRestAreEmpty) {
            for (Map.Entry<String, ImprovedReefscapeReefBranch> entry :
                    ((ImprovedArena2025Reefscape) SimulatedArena.getInstance()).blueReefSimulation.getBranches()) {
                if (!coralIds.contains("B" + entry.getKey())) {
                    corals.add("B" + entry.getKey() + "0");
                }
            }

            for (Map.Entry<String, ImprovedReefscapeReefBranch> entry :
                    ((ImprovedArena2025Reefscape) SimulatedArena.getInstance()).redReefSimulation.getBranches()) {
                if (!coralIds.contains("R" + entry.getKey())) {
                    corals.add("R" + entry.getKey() + "0");
                }
            }
        }

        StringBuilder sb = new StringBuilder();
        List<String> actualReefString = new ArrayList<>();

        for (String coral : corals) {
            coral = coral.trim();

            ImprovedReefscapeReefBranch branch;

            if (coral.charAt(0) == 'B') {
                branch = ((ImprovedArena2025Reefscape) SimulatedArena.getInstance())
                        .blueReefSimulation.getBranch(coral.substring(1, 3));
            } else if (coral.charAt(0) == 'R') {
                branch = ((ImprovedArena2025Reefscape) SimulatedArena.getInstance())
                        .redReefSimulation.getBranch(coral.substring(1, 3));
            } else {
                throw new IllegalArgumentException("Invalid reef alliance: " + coral);
            }

            if (branch == null) {
                throw new IllegalArgumentException("Invalid reef coral: " + coral);
            }

            int expectedCount;
            if (coral.length() >= 4) {
                expectedCount = Integer.parseInt(coral.substring(3));
            } else {
                expectedCount = 1; // Default to 1 if no count is specified
            }

            int actualCount = branch.getGamePieceCount();

            if (actualCount != expectedCount) {
                String branchId = coral.substring(0, 3);
                actualReefString.add(branchId + actualCount);
                sb.append("Expected branch " + branchId + " to have " + expectedCount + " coral, but it has "
                        + actualCount + ".\n");
            }
        }

        if (!sb.isEmpty()) {
            assertionFailure()
                    .message(sb.toString())
                    .expected(reefString)
                    .actual(String.join(",", actualReefString))
                    .buildAndThrow();
        }
    }
}
