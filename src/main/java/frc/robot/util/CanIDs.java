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
        ids.put("fl-encoder", 27);

        ids.put("fr-drive", 16);
        ids.put("fr-turn", 20);
        ids.put("fr-encoder", 22);

        ids.put("bl-drive", 13);
        ids.put("bl-turn", 12);
        ids.put("bl-encoder", 14);

        ids.put("br-drive", 41);
        ids.put("br-turn", 21);
        ids.put("br-encoder", 4);

        // ++++ Climber ++++
        ids.put("climber", 70); //TODO

        // ++++ Shooter ++++
        ids.put("shooter-1", 60); //TODO
        ids.put("shooter-2", 61); //TODO
        ids.put("shooter-pivot", 62); //TODO
        ids.put("shooter-arm", 63); //TODO

        // ++++ Intake ++++
        ids.put("intake-pivot", 50); //TODO
        ids.put("roller", 51); //TODO

        // ++++ Transfer ++++
        ids.put("transfer-motor", 80); //TODO

        // ++++ Random Test Thing ++++
        ids.put("testing-motor", 62); //TODO
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