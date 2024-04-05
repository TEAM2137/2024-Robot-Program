package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.CanIDs;
import frc.robot.util.PID;
import frc.robot.vision.VisionBlendedPoseEstimator;
import frc.robot.vision.VisionBlender;

// Everything in this file will be done in the order front left, front right, back left, back right
public class SwerveDrivetrain extends SubsystemBase {

    public static class Constants {
        public static final int gyroID = 5;

        public static final double length = Units.inchesToMeters(21.5);
        public static final double width = Units.inchesToMeters(21.5);

        public static final double driveMaxSpeed = 3.5;
        public static final double driveMaxAccel = 2.0;

        public static SwerveModuleConstants frontLeft = new SwerveModuleConstants(
            CanIDs.get("fl-drive"), 
            CanIDs.get("fl-turn"), 
            CanIDs.get("fl-encoder"), 
            -0.066650391, "Front Left");
        public static SwerveModuleConstants frontRight = new SwerveModuleConstants(
            CanIDs.get("fr-drive"), 
            CanIDs.get("fr-turn"), 
            CanIDs.get("fr-encoder"), 
            0.071044921, "Front Right");
        public static SwerveModuleConstants backLeft = new SwerveModuleConstants(
            CanIDs.get("bl-drive"), 
            CanIDs.get("bl-turn"), 
            CanIDs.get("bl-encoder"),
            -0.164306641, "Back Left");
        public static SwerveModuleConstants backRight = new SwerveModuleConstants(
            CanIDs.get("br-drive"),
            CanIDs.get("br-turn"), 
            CanIDs.get("br-encoder"),
            0.280761718, "Back Right");

        public static PID translationPIDConstants = new PID(0.5, 0, 0);

        public static PID teleopThetaPIDConstants = new PID(0.5, 0.0, 0.4);
        public static TrapezoidProfile.Constraints teleopThetaPIDConstraints = new TrapezoidProfile.Constraints(6, 4); // new

        public static PID autoThetaPIDConstants = new PID(2.5, 0, 0);
        public static TrapezoidProfile.Constraints autoThetaPIDConstraints = new TrapezoidProfile.Constraints(16, 16); // old

        public static PID purePIDTranslationConstants = new PID(0, 0, 0); // can be ignored

        public static class SwerveModuleConstants {
            public final int driveID;
            public final int turningID;
            public final int encoderID;

            public double offset;
            public final String moduleName;

            SwerveModuleConstants(int driveID, int turningID, int encoderID, double offsetDegrees, String moduleName) {
                this.driveID = driveID;
                this.turningID = turningID;
                this.encoderID = encoderID;
                this.offset = offsetDegrees;
                this.moduleName = moduleName;
            }
        }
    }

    SwerveDriveKinematics kinematics;
    
    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    private SwerveModule[] swerveArray;
    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    private Timer timer;
    private double lastTime;

    private Pigeon2 pigeonIMU;

    private VisionBlendedPoseEstimator poseEstimator;
    private VisionBlender vision;

    private Field2d field2d = new Field2d();

    private StructArrayPublisher<SwerveModuleState> swervePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();
    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Robot Pose", Pose2d.struct).publish();
    public StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Target Location", Pose2d.struct).publish();
    private StructPublisher<Rotation2d> rotationPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Robot Rotation", Rotation2d.struct).publish();

    /**
     * Creates a swerve drivetrain (uses values from constants)
     */
    public SwerveDrivetrain(ModuleType moduleType, VisionBlender vision) {
        this.vision = vision;

        // locations of all of the modules (for kinematics)
        Translation2d frontLeftLocation = new Translation2d(Constants.length / 2, Constants.width / 2);
        Translation2d frontRightLocation = new Translation2d(Constants.length / 2, -Constants.width / 2);
        Translation2d backLeftLocation = new Translation2d(-Constants.length / 2, Constants.width / 2);
        Translation2d backRightLocation = new Translation2d(-Constants.length / 2, -Constants.width / 2);

        // the kinematics object for converting chassis speeds to module rotations and powers
        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        // each of the modules
        if (moduleType == ModuleType.Neo) {
            frontLeftModule = new NeoModule(Constants.frontLeft);
            frontRightModule = new NeoModule(Constants.frontRight);
            backLeftModule = new NeoModule(Constants.backLeft);
            backRightModule = new NeoModule(Constants.backRight);
        } else {
            frontLeftModule = new FalconModule(Constants.frontLeft);
            frontRightModule = new FalconModule(Constants.frontRight);
            backLeftModule = new FalconModule(Constants.backLeft);
            backRightModule = new FalconModule(Constants.backRight);
        }

        // an array of the swerve modules, to make life easier
        swerveArray = new SwerveModule[]{frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        // the gyro
        pigeonIMU = new Pigeon2(Constants.gyroID, RobotContainer.getRioCanBusName());
        pigeonIMU.getConfigurator().apply(new Pigeon2Configuration());
        pigeonIMU.reset();

        // create pose estimator
        resetDriveDistances();
        updateModulePositions();
        poseEstimator = new VisionBlendedPoseEstimator(kinematics, getRotation(), modulePositions, vision);

        timer = new Timer();
        timer.reset();
        timer.start();
        
        resetGyro();
        resetOdometry();
    }

    public void init() {
        poseEstimator.init();
    }

    public void displayCurrentOffsets() {
        SmartDashboard.putNumber("Offset FL", swerveArray[0].encoderOffset);
        SmartDashboard.putNumber("Offset FR", swerveArray[1].encoderOffset);
        SmartDashboard.putNumber("Offset BL", swerveArray[2].encoderOffset);
        SmartDashboard.putNumber("Offset BR", swerveArray[3].encoderOffset);
    }

    /**
     * Periodic loop of the subsystem
     */
    @Override
    public void periodic() {

        updateOdometry();

        field2d.setRobotPose(getPose());

        SmartDashboard.putData("Field", field2d);

        swervePublisher.set(getSwerveModuleStates()); // AdvantageScope swerve states
        rotationPublisher.set(getRotation());
        posePublisher.set(getPose()); // AdvantageScope pose

        SmartDashboard.putNumber("Robot X", getPose().getX());
        SmartDashboard.putNumber("Robot Y", getPose().getY());
        SmartDashboard.putNumber("Robot Rotation", getPose().getRotation().getDegrees());
    }

    private void updateOdometry() {
        double time = timer.get();
        double dt = time - lastTime;
        lastTime = time;

        if (dt == 0) {
            return;
        }
        
        updateModulePositions();
        poseEstimator.update(getRotation(), modulePositions);
    }

    private void updateModulePositions() {
        boolean flipDistances = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

        double[] distances = new double[] {
            frontLeftModule.getDriveDistance() * (flipDistances ? -1 : 1),
            frontRightModule.getDriveDistance() * (flipDistances ? -1 : 1),
            backLeftModule.getDriveDistance() * (flipDistances ? -1 : 1),
            backRightModule.getDriveDistance() * (flipDistances ? -1 : 1)
        };

        modulePositions[0] = new SwerveModulePosition(distances[0], frontLeftModule.getModuleRotation());
        modulePositions[1] = new SwerveModulePosition(distances[1], frontRightModule.getModuleRotation());
        modulePositions[2] = new SwerveModulePosition(distances[2], backLeftModule.getModuleRotation());
        modulePositions[3] = new SwerveModulePosition(distances[3], backRightModule.getModuleRotation());

        SmartDashboard.putNumber("Module Pos 0", distances[0]);
        SmartDashboard.putNumber("Module Pos 1", distances[1]);
        SmartDashboard.putNumber("Module Pos 2", distances[2]);
        SmartDashboard.putNumber("Module Pos 3", distances[3]);
    }

    public void resetModuleAngles() {
        for (SwerveModule module : swerveArray) {
            module.homeTurningMotor();
        }
    }

    /**
     * @return the angle of the robot (CCW positive (normal))
     */
    public Rotation2d getRotation() {
        double raw = pigeonIMU.getYaw().getValueAsDouble(); // % 360;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return Rotation2d.fromDegrees(raw).plus(Rotation2d.fromDegrees(alliance == Alliance.Red ? 180 : 0));
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public double getThetaVelocity() {
        return pigeonIMU.getAngularVelocityZDevice().getValueAsDouble();
    }

    public void resetDriveDistances() {
        frontLeftModule.resetDriveEncoder();
        frontRightModule.resetDriveEncoder();
        backLeftModule.resetDriveEncoder();
        backRightModule.resetDriveEncoder();
    }

    public void resetGyro() {
        pigeonIMU.setYaw(0);
    }

    /**
     * @param speeds speed of the chassis with -1 to 1 on translation
     */
    public void driveTranslationRotationRaw(ChassisSpeeds speeds) {
        if(speeds.vxMetersPerSecond + speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond == 0) {
            // if power isn't being applied, don't set the module rotation to zero
            setAllModuleDriveRawPower(0);
            selfTargetAllModuleAngles();
        } else {
            //setDriveBrakeMode(speeds.omegaRadiansPerSecond > 0);

            // if power, drive it
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds); //convert speeds to individual modules

            SwerveDriveKinematics.desaturateWheelSpeeds(states, 1); //normalize speeds to be all between -1 and 1
            for (int i = 0; i < states.length; i++) {
                //optimize module rotation (instead of a >90 degree turn, turn less and flip wheel direction)
                states[i] = SwerveModuleState.optimize(states[i], swerveArray[i].getModuleRotation());

                //set all the things
                swerveArray[i].setTurningTarget(states[i].angle);
                swerveArray[i].setDrivePowerRaw(states[i].speedMetersPerSecond);
            }
        }
    }

    /**
     * @param speeds speed of the chassis in m/s and rad/s
     */
    public void driveTranslationRotationVelocity(ChassisSpeeds speeds) {
        if(speeds.vxMetersPerSecond + speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond == 0) {
            // if power isn't being applied, don't set the module rotation to zero
            for (int i = 0; i < swerveArray.length; i++) {
                swerveArray[i].setDriveVelocity(0);
                swerveArray[i].setTurningTarget(swerveArray[i].getModuleRotation());
            }
        } else {
            // if power, drive it
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds); //convert speeds to individual modules

            SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.driveMaxSpeed); //normalize speeds to be all between min and max speed
            for (int i = 0; i < states.length; i++) {
                //optimize module rotation (instead of a >90 degree turn, turn less and flip wheel direction)
                states[i] = SwerveModuleState.optimize(states[i], swerveArray[i].getModuleRotation());

                //set all the things
                swerveArray[i].setTurningTarget(states[i].angle);
                swerveArray[i].setDriveVelocity(states[i].speedMetersPerSecond);
            }
        }
    }

    /**
     * @return swerve module states in m/s
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < swerveArray.length; i++) {
            states[i] = swerveArray[i].getSwerveModuleState();
        }
        return states;
    }

    /**
     * @return the pose of the robot in meters
     */
    public Pose2d getPose() {
        Pose2d pose = poseEstimator.grabEstimatedPose();
        return new Pose2d(pose.getX(), pose.getY(), getRotation());
    }

    /**
     * For debugging
     * @param power power of the drive motors -1 to 1
     */
    public void setAllModuleDriveRawPower(double power) {
        frontLeftModule.setDrivePowerRaw(power);
        frontRightModule.setDrivePowerRaw(power);
        backLeftModule.setDrivePowerRaw(power);
        backRightModule.setDrivePowerRaw(power);
    }

    /**
     * For debugging
     * @param angle angle of all of the modules
     */
    public void setAllModuleRotations(Rotation2d angle) {
        frontLeftModule.setTurningTarget(angle);
        frontRightModule.setTurningTarget(angle);
        backLeftModule.setTurningTarget(angle);
        backRightModule.setTurningTarget(angle);
    }

    /**
     * For debugging
     * @param velocity target velocity in m/s
     */
    public void setAllModuleDriveVelocity(double velocity) {
        frontLeftModule.setDriveVelocity(velocity);
        frontRightModule.setDriveVelocity(velocity);
        backLeftModule.setDriveVelocity(velocity);
        backRightModule.setDriveVelocity(velocity);
    }

    public void selfTargetAllModuleAngles() {
        frontLeftModule.selfTargetAngle();
        frontRightModule.selfTargetAngle();
        backLeftModule.selfTargetAngle();
        backRightModule.selfTargetAngle();
    }

    /**
     * Resets the PoseEstimator to a specified pose
     */
    public void resetOdometry(Pose2d pose) {
        // This is necessary because PathPlanner flips the ROTATION of the starting pose as well
        // as the position for autons depending on the alliance, which we don't want.
        boolean reFlip = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

        poseEstimator.resetPosition(getRotation(), new Pose2d(pose.getX(), pose.getY(),
            pose.getRotation().plus(Rotation2d.fromDegrees(reFlip ? 180 : 0))), modulePositions);
        updateOdometry();
    }

    /**
     * Resets the PoseEstimator to a specified position with the gyro rotation
     */
    public void resetOdometry(Translation2d translation) {
        resetOdometry(new Pose2d(translation, getRotation()));
    }

    /**
     * Resets the PoseEstimator to (0, 0) with the gyro rotation
     */
    private void resetOdometry() {
        resetOdometry(new Pose2d(new Translation2d(), getRotation()));
    }

    /**
     * Sets the PoseEstimator position to wherever the limelight thinks the
     * robot currently is
     */
    public void visionResetOdometry() {
        resetOdometry(new Pose2d(vision.getBlendedPose().getTranslation(), getRotation()));
    }

    /**
     * Sets the drivetrain into x-locking mode, making it defense resistant
     */
    public void xLock() {
        double length = Constants.length / 2;
        double width = Constants.width / 2;
        setAllModuleDriveRawPower(0);

        frontLeftModule.setTurningTarget(new Rotation2d(Math.atan2(width, length)));
        frontRightModule.setTurningTarget(new Rotation2d(Math.atan2(-width, length)));
        backLeftModule.setTurningTarget(new Rotation2d(Math.atan2(width, -length)));
        backRightModule.setTurningTarget(new Rotation2d(Math.atan2(-width, -length)));
    }

    public TrajectoryConfig getDefaultConstraint() {
        return new TrajectoryConfig(Constants.driveMaxSpeed, Constants.driveMaxAccel).setKinematics(kinematics);
    }

    public void setDriveBrakeMode(boolean brake) {
        this.frontLeftModule.setDriveMode(brake);
        this.backLeftModule.setDriveMode(brake);
        this.frontRightModule.setDriveMode(brake);
        this.backRightModule.setDriveMode(brake);
    }

    public void setTurnBrakeMode(boolean brake) {
        this.frontLeftModule.setTurnBrakeMode(brake);
        this.frontRightModule.setTurnBrakeMode(brake);
        this.backLeftModule.setTurnBrakeMode(brake);
        this.backRightModule.setTurnBrakeMode(brake);
    }

    public void setField2dTrajectory(Trajectory trajectory) {
        field2d.getObject("path").setTrajectory(trajectory);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public enum ModuleType {
        Neo,
        Falcon
    }
}