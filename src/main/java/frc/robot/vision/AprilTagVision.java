package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AprilTagVision {
    
    private NetworkTable table;
    private NetworkTableEntry botpose;
    private double[] pose;

    private double posX;
    private double posY;

    private double rotation;

    public AprilTagVision() {
        table = NetworkTableInstance.getDefault().getTable("limelight-a");
        botpose = table.getEntry("botpose");
    }

    /**
     * Updates the position of the robot using the aprilTags
     */
    public void updateValues() {
        pose = botpose.getDoubleArray(new double[6]);
        if (pose.length == 0) pose = new double[6];
        
        posX = pose[0];
        posY = pose[1];
        rotation = pose[5];
    
        // Post botpose to smart dashboard
        SmartDashboard.putNumber("Field Position X", posX);
        SmartDashboard.putNumber("Field Position Y", posY);
        SmartDashboard.putNumber("Robot Rotation", rotation);
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
    public double getRotation() {
        return rotation;
    }

    /**
     * @return true if the limelight is detecting an aprilTag, false if it isn't
     */
    public boolean hasTarget() {
        pose = botpose.getDoubleArray(new double[6]);
        if (pose.length == 0 || pose[0] == 0) return false;
        return true;
    }
}
