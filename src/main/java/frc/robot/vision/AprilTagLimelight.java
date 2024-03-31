package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagLimelight {
    
    private NetworkTable table;
    private NetworkTableEntry botposeEntry;
    
    private double[] pose;
    private Rotation2d rotation;
    private double posX;
    private double posY;

    private double latency;

    public AprilTagLimelight(String name) {
        table = NetworkTableInstance.getDefault().getTable(name);
        resetAlliance();
    }

    public void resetAlliance() {
        botposeEntry = table.getEntry("botpose_wpiblue");
    }

    /**
     * Updates the position of the robot using the aprilTags
     */
    public void updateValues() {
        pose = botposeEntry.getDoubleArray(new double[6]);
        if (pose.length == 0) pose = new double[6];
        
        posX = pose[0];
        posY = pose[1];
        rotation = Rotation2d.fromDegrees(pose[5]);
        latency = pose[6];
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
        pose = botposeEntry.getDoubleArray(new double[6]);
        if (pose.length == 0 || pose[0] == 0) return false;
        return true;
    }

    public Pose2d getPose() {
        return new Pose2d(getX(), getY(), getRotation());
    }
}