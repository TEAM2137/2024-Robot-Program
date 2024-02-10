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
        ids.put("climber", 32); //TODO

        // ++++ Shooter ++++
        ids.put("shooter-1", 33); //TODO
        ids.put("shooter-2", 34); //TODO
        ids.put("shooter-pivot", 35); //TODO

        // ++++ Intake ++++
        ids.put("intake-pivot", 36); //TODO
        ids.put("roller", 37); //TODO

        // ++++ Transfer ++++
        ids.put("transfer-motor", 38); //TODO

        // ++++ Trapper ++++
        ids.put("trapper-motor", 39); //TODO
        ids.put("trapper-pivot", 40); //TODO
        ids.put("trapper-arm", 42); //TODO

        // ++++ Random Test Thing ++++
        ids.put("testing-motor", 62);
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