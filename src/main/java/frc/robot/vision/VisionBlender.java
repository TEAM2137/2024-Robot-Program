package frc.robot.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionBlender {
    public ArrayList<AprilTagLimelight> limelights = new ArrayList<>();
    public Pose2d lastPose;

    public static final int POSE_GRABS = 3;
    public List<Pose2d> previousPoses = new ArrayList<>(3);

    public VisionBlender(AprilTagLimelight... limelights) {
        this.limelights.addAll(Arrays.asList(limelights));
    }

    public Pose2d getAveragedPose() {
        if (!hasTarget() || limelights == null) return null;
        double averageX = 0;
        double averageY = 0;
        double averageRot = 0;

        for (Pose2d pose : previousPoses) {
            if (pose == null || pose.getRotation() == null) previousPoses.remove(pose);
            averageX += pose.getX();
            averageY += pose.getY();
            averageRot += pose.getRotation().getDegrees();
        }

        averageX /= previousPoses.size();
        averageY /= previousPoses.size();
        averageRot /= previousPoses.size();

        previousPoses.clear();
        return new Pose2d(averageX, averageY, new Rotation2d(averageRot));
    }

    public boolean hasPose() {
        return previousPoses.size() >= POSE_GRABS;
    }

    public Pose2d updatePose() {
        if (limelights == null || !hasTarget()) return null;
        Pose2d pose = limelights.get(0).getPose();
        if (pose == null) return null;
        
        previousPoses.add(pose);
        return pose;
    }

    public Pose2d getPose() {
        if (limelights == null) return null;

        Pose2d pose = limelights.get(0).getPose();
        if (pose == null) return null;

        if (lastPose == null) {
            lastPose = pose;
            return pose;
        }

        // // Calculate the error between the poses
        // double diffX = lastPose.getX() - pose.getX();
        // double diffY = lastPose.getY() - pose.getY();
        // double dst = Math.hypot(diffX, diffY);

        // // Invalidate the incoming pose if it is innacurate
        // if (dst > 0.4) {
        //     return null;
        // }

        // Otherwise, use the incoming pose
        lastPose = pose;
        return pose;
    }

    /**
     * @return the current limelight timestamp accounted for latency
     */
    public double getTimestamp() {
        if (limelights == null) return Timer.getFPGATimestamp();
        return Timer.getFPGATimestamp();// - limelights.get(0).getLatency() / 1000.0;
    }

    public boolean hasTarget() {
        if (limelights == null) {
            lastPose = null;
            return false;
        }

        for (AprilTagLimelight limelight : limelights) {
            if (limelight.hasTarget()) return true;
        }

        lastPose = null;
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

        Pose2d pose = getPose();
        if (pose == null || pose.getRotation() == null) return;

        // Post botpose to smart dashboard
        SmartDashboard.putNumber("LL-PositionX", pose.getX());
        SmartDashboard.putNumber("LL-PositionY", pose.getY());
        SmartDashboard.putNumber("LL-Rotation", pose.getRotation().getDegrees());
    }
}
