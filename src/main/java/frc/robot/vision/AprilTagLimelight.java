package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagLimelight {

    private String name;
    
    private NetworkTable table;
    private NetworkTableEntry botposeEntry;
    private NetworkTableEntry orientationEntry;
    private NetworkTableEntry latencyEntry;
    
    private double[] poseArray;
    private Rotation2d rotation;
    private double posX;
    private double posY;

    private double latency;

    public AprilTagLimelight(String name) {
        this.name = name;
        table = NetworkTableInstance.getDefault().getTable(name);
        resetAlliance();
    }

    public void resetAlliance() {
        botposeEntry = table.getEntry("botpose_orb_wpiblue"); // Use the new botpose
        orientationEntry = table.getEntry("robot_orientation_set"); // 2024.5.0 requires robot orientation
        latencyEntry = table.getEntry("tl");
    }

    /**
     * Updates the position of the robot using the aprilTags
     */
    public void updateValues(Rotation2d fieldRotation, double rotationRate) {
        // Update pose
        poseArray = botposeEntry.getDoubleArray(new double[6]);
        if (poseArray.length == 0) poseArray = new double[6];
        posX = poseArray[0];
        posY = poseArray[1];
        rotation = Rotation2d.fromDegrees(poseArray[5]);

        // Update latency
        latency = latencyEntry.getDouble(0);

        // Post orientation
        double[] orientationArray = new double[6];
        orientationArray[0] = fieldRotation.getDegrees();
        orientationArray[1] = rotationRate;
        orientationArray[2] = 0;
        orientationArray[3] = 0;
        orientationArray[4] = 0;
        orientationArray[5] = 0;
        orientationEntry.setDoubleArray(orientationArray);
    }

    public String getName() {
        return name;
    }

    /**
     * @return The raw latency of the limelight images
     */
    public double getLatency() {
        return latency;
    }

    /**
     * @return the x position of the robot relative to the bottom left corner of the field (inches)
     */
    public double getX() {
        return posX;
    }
    
    /**
     * @return the y position of the robot relative to the bottom left corner of the field (inches)
     */
    public double getY() {
        return posY;
    }

    /**
     * @return the z rotation of the robot (degrees)
     */
    public Rotation2d getRotation() {
        return rotation;
    }

    /**
     * @return true if the limelight is detecting an aprilTag, false if it isn't
     */
    public boolean hasTarget() {
        poseArray = botposeEntry.getDoubleArray(new double[6]);
        if (poseArray.length == 0 || poseArray[0] == 0) return false;
        return true;
    }

    public Pose2d getPoseArray() {
        return new Pose2d(getX(), getY(), getRotation());
    }
}