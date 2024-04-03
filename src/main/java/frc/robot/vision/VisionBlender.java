package frc.robot.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionBlender {
    private ArrayList<AprilTagLimelight> limelights = new ArrayList<>();

    /**
     * Creates a new vision blender from a list of limelights
     */
    public VisionBlender(List<AprilTagLimelight> limelights) {
        this.limelights.addAll(limelights);
    }

    /**
     * Creates a new vision blender from limelight hostnames
     */
    public VisionBlender(String... hostnames) {
        this(mapHostnames(Arrays.asList(hostnames)));
    }

    static List<AprilTagLimelight> mapHostnames(List<String> hostnames) {
        List<AprilTagLimelight> limelightList = new ArrayList<>();
        hostnames.forEach(hostname -> limelightList.add(new AprilTagLimelight(hostname)));
        return limelightList;
    }

    /**
     * ( ! ) Make sure to call updateValues() regularly for this method to work properly
     * @return The blended pose of all the limelights that are displaying valid poses
     */
    public Pose2d getBlendedPose() {
        if (limelights == null || limelights.size() == 0) return null;

        ArrayList<Pose2d> currentPoses = new ArrayList<>(limelights.size());

        // Add all valid vision poses to the list
        limelights.forEach(limelight -> {
            if (limelight != null && limelight.hasTarget()) {
                Pose2d grabbedPose = limelight.getPose();
                if (grabbedPose != null) currentPoses.add(grabbedPose);
            }
        });

        if (currentPoses.size() <= 0) return null;

        Translation2d translation = new Translation2d();
        Rotation2d rotation = new Rotation2d();

        // Average out all of the limelight poses
        for (Pose2d pose : currentPoses) {
            translation = translation.plus(pose.getTranslation());
            rotation = rotation.plus(pose.getRotation());
        }

        translation = translation.div(currentPoses.size());
        rotation = rotation.div(currentPoses.size());

        // Create a new pose with the averaged values
        return new Pose2d(translation, rotation);
    }

    /**
     * @return the current limelight timestamp accounted for latency
     */
    public double getTimestamp() {
        if (limelights == null) return Timer.getFPGATimestamp();
        return Timer.getFPGATimestamp() - limelights.get(0).getLatency() / 1000.0;
    }

    /**
     * @return true if any of the limelights have a target
     */
    public boolean hasTarget() {
        if (limelights == null) return false;

        for (AprilTagLimelight limelight : limelights) {
            if (limelight.hasTarget()) return true;
        }

        return false;
    }

    /**
     * Grabs and updates the data from each of the limelights
     */
    public void updateValues() {
        if (limelights == null) return;
        limelights.forEach((limelight) -> {
            if (limelight.hasTarget()) limelight.updateValues();
        });

        Pose2d pose = getBlendedPose();
        if (pose == null || pose.getRotation() == null) return;

        // Post botpose to smart dashboard
        SmartDashboard.putNumber("LL-PositionX", pose.getX());
        SmartDashboard.putNumber("LL-PositionY", pose.getY());
        SmartDashboard.putNumber("LL-Rotation", pose.getRotation().getDegrees());
    }

    public static boolean isInField(Translation2d pos) {
        if (pos.getX() < 0 || pos.getX() > 16.5) return false;
        if (pos.getY() < 0 || pos.getY() > 8.1) return false;
        return true;
    }

    /**
     * Sets the current botpose entry of the limelights to the correct alliance
     */
    public void resetAlliances() {
        limelights.forEach(limelight -> limelight.resetAlliance());
    }
}
