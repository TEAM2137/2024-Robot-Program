package frc.robot.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * This class is kind of useless right now, I'm working on it
 */
public class VisionBlendedPoseEstimator {
    public SwerveDrivePoseEstimator poseEstimator;
    public VisionBlender visionBlender;

    public VisionBlendedPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions, VisionBlender visionBlender) {
        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, new Pose2d());
        this.visionBlender = visionBlender;
    }

    public void update(double time, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        visionBlender.updateValues();
        poseEstimator.addVisionMeasurement(visionBlender.getPose(), visionBlender.getTimestamp());
        poseEstimator.updateWithTime(time, gyroAngle, modulePositions);
    }

    public Pose2d grabEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPosition(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d pose2d) {
        poseEstimator.resetPosition(rotation, modulePositions, pose2d);
    }
}
