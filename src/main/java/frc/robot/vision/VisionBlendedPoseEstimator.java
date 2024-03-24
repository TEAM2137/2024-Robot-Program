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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class VisionBlendedPoseEstimator {
    public static class Constants {
        private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
        private static final Vector<N3> visionStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(20));
    }

    public SwerveDrivePoseEstimator poseEstimator;
    public VisionBlender visionBlender;

    /**
     * Creates a new vision-blended swerve pose estimator
     * @param kinematics kinematics of the swerve drivetrain
     * @param gyroAngle the current angle ofthe gyro
     * @param modulePositions the current positions of the modules
     * @param visionBlender the vision blender to use for AprilTag data
     */
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
        if (visionBlender.hasTarget()) {
            Pose2d visionPose = visionBlender.getPose();
            if (visionPose != null && ((DriverStation.isTeleop() && RobotContainer.getInstance().teleop != null && RobotContainer.getInstance().teleop.isTargeting()) ||
                    (RobotContainer.getInstance().auto != null && RobotContainer.getInstance().auto.isTargetingEnabled()))) {
                visionPose = new Pose2d(visionPose.getX(), visionPose.getY(), gyroAngle);
                poseEstimator.addVisionMeasurement(visionPose, visionBlender.getTimestamp());
            }
        }
        
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyroAngle, modulePositions);
    }

    /**
     * @return the poseEstimator's estimated position
     */
    public Pose2d grabEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * @param rotation the gyro rotation to reset to
     * @param pose2d the pose of the robot to reset to
     * @param modulePositions current positions of the modules
     */
    public void resetPosition(Rotation2d rotation, Pose2d pose2d, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(rotation, modulePositions, pose2d);
    }
}
