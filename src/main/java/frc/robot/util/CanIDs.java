package frc.robot.util;

import java.util.HashMap;

/** Static class for easy access and changing of can IDs.
 * <p>Use <code>CanIDs.get("example-id")</code> to get a named ID.
 */
public class CanIDs {
    private static HashMap<String, Integer> ids = new HashMap<>();

    // Add new can IDs here!
    static {
        // ++++ Drivetrain ++++

        ids.put("fl-drive", 10);
        ids.put("fl-turn", 31);
        ids.put("fl-encoder", 4);

        ids.put("fr-drive", 16);
        ids.put("fr-turn", 20);
        ids.put("fr-encoder", 14);

        ids.put("bl-drive", 13);
        ids.put("bl-turn", 12);
        ids.put("bl-encoder", 22);

        ids.put("br-drive", 41);
        ids.put("br-turn", 21);
        ids.put("br-encoder", 27);

        // ++++ Climber ++++
        ids.put("climber", 70); //TODO

        // ++++ Shooter ++++
        ids.put("launcher-1", 60); //TODO
        ids.put("launcher-2", 61); //TODO

        // ++++ Intake ++++
        ids.put("intake-pivot", 50); //TODO
        ids.put("roller", 51); //TODO

        // ++++ Transfer ++++
        ids.put("transfer-motor", 80); //TODO
    }

    /**
     * @param id The name of the can ID to get
     * @return The can id value that maps to the name. Returns -1 if the id doesn't exist in the list.
     */
    public static int get(String id) {
        if (!ids.containsKey(id)) return -1;
        return ids.get(id);
    }
}