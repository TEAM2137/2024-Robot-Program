package frc.robot.subsystems.swerve.positioning;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
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

    private StructPublisher<Pose2d> autoStartPose = NetworkTableInstance.getDefault()
        .getStructTopic("Auto Start Pose", Pose2d.struct).publish();

    public RobotPositioner(SwerveDrivetrain drivetrain, int gyroID, SwerveDriveKinematics kinematics, SwerveModulePosition[] modulePositions, VisionBlender vision) {
        this.drivetrain = drivetrain;

        pigeon = new Pigeon2(gyroID, RobotContainer.getRioCanBusName());
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.reset();

        poseEstimator = new VisionPoseEstimator(drivetrain.getKinematics(),
            getRotation(Perspective.Field), modulePositions, vision);
        resetPerspective();
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
    public Pose2d getFieldPose() {
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

    /**
     * @return The blue origin X-coordinate of the robot
     */
    public double getX() {
        return getFieldPose().getX();
    }

    /**
     * @return The blue origin Y-coordinate of the robot
     */
    public double getY() {
        return getFieldPose().getY();
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
     * Resets the perspective of both the module state positioning and
     * the gyro to the current state of the robot
     */
    public void resetPerspective() {
        resetDriveDistances();
        resetGyro();
        resetOdometry();
    }

    public void resetDriveDistances() {
        drivetrain.resetDriveDistances();
        drivetrain.updateModulePositions();
    }

    /**
     * PathPlanner specific method for resetting odometry
     */
    public void setPathplannerOdometry(Pose2d pose) {
        autoStartPose.set(pose);
        
        // To handle non-zero starting angles
        boolean flip = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        setGyro(pose.getRotation().getDegrees() - (flip ? 180 : 0));
        
        resetDriveDistances();
        poseEstimator.resetPosition(getRotation(Perspective.Driver), pose, drivetrain.getModulePositions());
    }

    /**
     * Resets the Pose Estimator to a specified field pose
     */
    public void resetOdometry(Pose2d pose) {
        System.out.println("Reset odometry to " + pose);
        poseEstimator.resetPosition(getRotation(Perspective.Field), new Pose2d(pose.getX(), pose.getY(),
            pose.getRotation()), drivetrain.getModulePositions());
    }

    /**
     * Resets the Pose Estimator to a specified position with the gyro rotation
     */
    public void resetOdometry(Translation2d translation) {
        resetOdometry(new Pose2d(translation, getRotation(Perspective.Field)));
    }

    /**
     * Resets the Pose Estimator to the current position and rotation of the robot
     */
    private void resetOdometry() {
        resetOdometry(getFieldPose());
    }

    /**
     * Updates the positioner with all the robot data
     * @param modulePositions
     */
    public void update(SwerveModulePosition[] modulePositions) {
        poseEstimator.update(getRotation(Perspective.Field), modulePositions);
    }

    public enum Perspective { Driver, Field }
}
