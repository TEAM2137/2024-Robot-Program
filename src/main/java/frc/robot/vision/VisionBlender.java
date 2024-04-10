package frc.robot.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;

public class VisionBlender {
    private HashMap<AprilTagLimelight, StructPublisher<Pose2d>> posePublishers = new HashMap<>();
    private ArrayList<AprilTagLimelight> limelights = new ArrayList<>();

    /**
     * Creates a new vision blender from a list of limelights
     */
    public VisionBlender(List<AprilTagLimelight> limelights) {
        this.limelights.addAll(limelights);

        for (AprilTagLimelight limelight : limelights) {
            posePublishers.put(limelight, NetworkTableInstance.getDefault()
                .getStructTopic("VisionPose-" + limelight.getName(), Pose2d.struct).publish());
        }
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
                Pose2d grabbedPose = limelight.getPoseArray();
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
     * @return true if any of the limelights have a target
     */
    public boolean hasTarget() {
        if (limelights == null || limelights.isEmpty()) return false;

        for (AprilTagLimelight limelight : limelights) {
            if (limelight.hasTarget()) return true; }

        return false;
    }

    /**
     * Grabs and updates the data from each of the limelights
     */
    public void updateValues(Rotation2d fieldRotation, double rotationRate) {
        if (limelights == null) return;
        limelights.forEach(limelight -> {
            if (limelight.hasTarget()) limelight.updateValues(fieldRotation, rotationRate);
        });
    }

    /**
     * Sets the current botpose entry of the limelights to the correct alliance
     */
    public void resetAlliances() {
        limelights.forEach(limelight -> limelight.resetAlliance());
    }

    /**
     * @return A list of all the semi-valid readings from the limelights that are
     * currently seeing april tags
     */
    public ArrayList<VisionReading> getReadings() {
        ArrayList<VisionReading> readings = new ArrayList<>();

        limelights.forEach(limelight -> {
            if (!limelight.hasTarget()) return;

            Pose2d visionPose = limelight.getPoseArray();
            if (visionPose == null) return;

            readings.add(new VisionReading(visionPose.getX(), visionPose.getY(),
                limelight.getLatency(), limelight));
        });

        return readings;
    }

    /**
     * Posts a vision pose from a limelight to the Network Tables
     * @param visionPose The pose to be added to the table
     * @param limelight The limelight that the pose belongs to
     */
    public void postLimelightPose(Pose2d visionPose, AprilTagLimelight limelight) {
        if (!posePublishers.containsKey(limelight)) return;
        StructPublisher<Pose2d> publisher = posePublishers.get(limelight);
        publisher.set(visionPose);
    }

    /**
     * A class that stores data from a single reading of a limelight
     */
    public class VisionReading {
        // TODO tune this better
        private static final double LATENCY_CUTOFF = 84.0;

        private final double x, y;
        private final double latency;
        private final AprilTagLimelight limelight;

        public VisionReading(double x, double y, double latency, AprilTagLimelight limelight) {
            this.x = x; this.y = y;
            this.latency = latency;
            this.limelight = limelight;
        }

        public double getX() { return x; }
        public double getY() { return y; }

        public double getLatency() { return latency; }
        
        public AprilTagLimelight getLimelight() { return limelight; }

        public double getTimestamp() {
            return Timer.getFPGATimestamp() - latency / 1000.0;
        }

        public boolean isRecent() {
            return latency < LATENCY_CUTOFF;
        }

        public boolean isInField() {
            if (x < 0 || x > 16.5) return false;
            if (y < 0 || y > 8.1) return false;
            return true;
        }
    }
}
