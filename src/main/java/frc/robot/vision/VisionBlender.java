package frc.robot.vision;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionBlender {
    public ArrayList<AprilTagLimelight> limelights;

    public VisionBlender(AprilTagLimelight... limelights) {
        this.limelights.addAll(Arrays.asList(limelights));
    }

    // TODO
    public Pose2d getPose() {
        return limelights.get(0).getPose();
    }

    /**
     * @return the current limelight timestamp accounted for latency
     */
    public double getTimestamp() {
        return Timer.getFPGATimestamp() - limelights.get(0).getLatency() / 1000.0;
    }

    /**
     * Grabs and updates the data from each of the limelights
     */
    public void updateValues() {
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
