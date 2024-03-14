package frc.robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class VisionBlendedPoseEstimator {

    public static class Constants {
        private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
        private static final Vector<N3> visionStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(20));
    }

    public SwerveDrivePoseEstimator poseEstimator;
    public VisionBlender visionBlender;

    public VisionBlendedPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions, VisionBlender visionBlender) {

        this.visionBlender = visionBlender;
        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions,
            new Pose2d(), Constants.stateStdDevs, Constants.visionStdDevs);
    }

    /**
     * Updates the poseEstimator with new vision values and swerve module positions
     * @param gyroAngle the measured angle of the gyro
     * @param modulePositions the current positions of the swerve modules
     */
    public void update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        visionBlender.updateValues();
        poseEstimator.addVisionMeasurement(visionBlender.getPose(), visionBlender.getTimestamp());
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyroAngle, modulePositions);
    }

    /**
     * @return the poseEstimator's estimated position
     */
    public Pose2d grabEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPosition(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d pose2d) {
        poseEstimator.resetPosition(rotation, modulePositions, pose2d);
    }
}
