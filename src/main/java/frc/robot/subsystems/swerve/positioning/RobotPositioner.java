package frc.robot.subsystems.swerve.positioning;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.vision.VisionBlender;
import frc.robot.vision.VisionPoseEstimator;

/**
 * A class responsible for handling everything position and
 * rotation related for the swerve drivetrain
 */
public class RobotPositioner {
    
    private SwerveDrivetrain drivetrain;
    private Pigeon2 pigeon;
    private VisionPoseEstimator poseEstimator;

    public RobotPositioner(SwerveDrivetrain drivetrain, int gyroID, SwerveDriveKinematics kinematics, SwerveModulePosition[] modulePositions, VisionBlender vision) {
        this.drivetrain = drivetrain;

        pigeon = new Pigeon2(gyroID, RobotContainer.getRioCanBusName());
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.reset();

        resetPerspective();
        poseEstimator = new VisionPoseEstimator(drivetrain.getKinematics(),
            getRotation(Perspective.Field), modulePositions, vision);
    }

    /**
     * The current rotation of the robot as of the specified perspective
     * @param perspective The perspective that the rotation is viewed from.
     * 
     * <p>For {@code Perspective.Field}, a rotation of 0 degrees is in the direction of
     * the blue alliance and 180 is in the direction of the red alliance.
     * 
     * <p>For {@code Perspective.Driver}, a rotation of 0 degrees is always facing away
     * from the driver station's alliance on the field, and 180 is facing towards it.
     * 
     * @return A {@code Rotation2d} representing the rotation
     */
    public Rotation2d getRotation(Perspective perspective) {
        double raw = pigeon.getYaw().getValueAsDouble();
        Rotation2d rot = Rotation2d.fromDegrees(raw);

        // Flip rotation as necessary
        if (perspective == Perspective.Driver) return rot;
        else {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            return rot.plus(Rotation2d.fromDegrees(alliance == Alliance.Red ? 180 : 0));
        }
    }

    /**
     * @return The current field perspective pose of the robot in meters
     */
    public Pose2d getPose() {
        Pose2d pose = poseEstimator.grabEstimatedPose();
        return new Pose2d(pose.getX(), pose.getY(), getRotation(Perspective.Field));
    }

    /**
     * @return The current driver perspective pose of the robot in meters
     */
    public Pose2d getDriverPose() {
        Pose2d pose = poseEstimator.grabEstimatedPose();
        return new Pose2d(pose.getX(), pose.getY(), getRotation(Perspective.Driver));
    }

    public double getX() {
        return getPose().getX();
    }

    public double getY() {
        return getPose().getY();
    }

    /**
     * Resets the pigeon gyro so zero degrees represents the current
     * angle of the robot
     */
    public void resetGyro() { pigeon.reset(); }

    /**
     * Sets the pigeon gyro so the specified degrees represents the
     * current angle of the robot
     * @param degrees The rotation in degrees to be set as the current angle
     */
    public void setGyro(double degrees) { setGyro(Rotation2d.fromDegrees(degrees)); }

    /**
     * Sets the pigeon gyro so the specified degrees represents the
     * current angle of the robot
     * @param rotation The rotation to be set as the current angle
     */
    public void setGyro(Rotation2d rotation) { pigeon.setYaw(rotation.getDegrees()); }

    /**
     * Resets the perspective of the module state positioning and
     * the gyro the the current state of the robot
     */
    public void resetPerspective() {
        drivetrain.resetDriveDistances();
        drivetrain.updateModulePositions();
        resetGyro();
        resetOdometry();
    }

    public void setPathplannerOdometry(Pose2d pose) {
        boolean flip = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

        // To handle non-zero starting angles
        setGyro(pose.getRotation().getDegrees());

        resetOdometry(new Pose2d(pose.getX(), pose.getY(), pose.getRotation()
            .plus(Rotation2d.fromDegrees(flip ? 180 : 0))));
    }

    /**
     * Resets the PoseEstimator to a specified field pose
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(Perspective.Field), new Pose2d(pose.getX(), pose.getY(),
            pose.getRotation()), drivetrain.getModulePositions());
    }

    /**
     * Resets the PoseEstimator to a specified position with the gyro rotation
     */
    public void resetOdometry(Translation2d translation) {
        resetOdometry(new Pose2d(translation, getRotation(Perspective.Field)));
    }

    /**
     * Resets the PoseEstimator to (0, 0) with the gyro rotation
     */
    private void resetOdometry() {
        resetOdometry(new Pose2d(new Translation2d(), getRotation(Perspective.Field)));
    }

    /**
     * Updates the positioner with all the robot data
     * @param modulePositions
     */
    public void update(SwerveModulePosition[] modulePositions) {
        poseEstimator.update(getRotation(Perspective.Driver), modulePositions);
    }

    public enum Perspective { Driver, Field }
}
