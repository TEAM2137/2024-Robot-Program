package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AprilTagVision {
    
    private NetworkTable table;
    private NetworkTableEntry botpose;
    private double[] pose;

    public final double fieldSizeX = 651.22;
    public final double fieldSizeY = 319;

    private double posX;
    private double posY;

    private double velocityX;
    private double velocityY;

    private double lastPosX;
    private double lastPosY; 

    private double rotation;

    public void init() {
        table = NetworkTableInstance.getDefault().getTable("limelight-0");
        botpose = table.getEntry("botpose");
    }

    /**
     * Updates the position of the robot using the aprilTags
     */
    public void updateValues() {
        pose = botpose.getDoubleArray(new double[6]);

        double poseX = pose[0];
        double poseY = pose[1];

        rotation = pose[5];

        if (!(poseX == 0.0 && poseY == 0.0)) {
            posX = (poseX * 39.37) + (fieldSizeX / 2);
            posY = (poseY * 39.37) + (fieldSizeY / 2);

            velocityX = posX - lastPosX;
            velocityY = posY - lastPosY;

            lastPosX = posX;
            lastPosY = posY;
        }
    
        // Post botpose to smart dashboard
        SmartDashboard.putNumber("Field Position X", poseX);
        SmartDashboard.putNumber("Field Position Y", poseY);

        SmartDashboard.putNumber("Velocity X", velocityX);
        SmartDashboard.putNumber("Velocity Y", velocityY);

        SmartDashboard.putNumber("Robot Rotation", rotation);
    }

    /**
     * @return the x position of the robot relative to the bottom left corner of the field (inches)
     */
    public double getX() {
        return posX;
    }

    /**
     * @return the amount the robot moved on the x axis since the last update (inches)
     */
    public double getXVelocity() {
        return velocityX;
    }
    
    /**
     * @return the y position of the robot relative to the bottom left corner of the field (inches)
     */
    public double getY() {
        return posY;
    }

    /**
     * @return the amount the robot moved on the y axis since the last update (inches)
     */
    public double getYVelocity() {
        return velocityY;
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
        if (pose[0] == 0.0) {
            return false;
        }
        return true;
    }
}
