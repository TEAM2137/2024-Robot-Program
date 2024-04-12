package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.positioning.RobotPositioner;
import frc.robot.subsystems.swerve.positioning.RobotPositioner.Perspective;
import frc.robot.util.CanIDs;
import frc.robot.vision.VisionBlender;

// Everything in this file will be done in the order front left, front right, back left, back right
public class SwerveDrivetrain extends SubsystemBase {

    public static class Constants {
        public static final int gyroID = 5;

        public static final double length = Units.inchesToMeters(21.5);
        public static final double width = Units.inchesToMeters(21.5);

        public static final double driveMaxSpeed = 3.5;
        public static final double driveMaxAccel = 3.5;

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

    public RobotPositioner positioner;

    private StructArrayPublisher<SwerveModuleState> swervePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();

    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Robot Pose", Pose2d.struct).publish();

    public StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Target Location", Pose2d.struct).publish();

    private StructPublisher<Rotation2d> fieldRotPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Field Space Rotation", Rotation2d.struct).publish();

    private StructPublisher<Rotation2d> driverRotPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Driver Space Rotation", Rotation2d.struct).publish();

    /**
     * Creates a swerve drivetrain (uses values from constants)
     */
    public SwerveDrivetrain(ModuleType moduleType, VisionBlender vision) {
        // Locations of all of the modules (for kinematics)
        Translation2d frontLeftLocation = new Translation2d(Constants.length / 2, Constants.width / 2);
        Translation2d frontRightLocation = new Translation2d(Constants.length / 2, -Constants.width / 2);
        Translation2d backLeftLocation = new Translation2d(-Constants.length / 2, Constants.width / 2);
        Translation2d backRightLocation = new Translation2d(-Constants.length / 2, -Constants.width / 2);

        // the kinematics object for converting chassis speeds to module rotations and powers
        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

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

        swerveArray = new SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};  

        updateModulePositions();
        positioner = new RobotPositioner(this, Constants.gyroID, kinematics, modulePositions, vision);
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

        swervePublisher.set(getSwerveModuleStates());
        fieldRotPublisher.set(positioner.getRotation(Perspective.Field));
        driverRotPublisher.set(positioner.getRotation(Perspective.Driver));
        posePublisher.set(positioner.getFieldPose());

        SmartDashboard.putNumber("Robot X", positioner.getX());
        SmartDashboard.putNumber("Robot Y", positioner.getY());
        SmartDashboard.putNumber("Robot Rotation", positioner.getRotation(Perspective.Driver).getDegrees());
    }

    private void updateOdometry() {
        updateModulePositions();
        positioner.update(modulePositions);
    }

    public void updateModulePositions() {
        boolean flipDistances = false;

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

    public void resetDriveDistances() {
        frontLeftModule.resetDriveEncoder();
        frontRightModule.resetDriveEncoder();
        backLeftModule.resetDriveEncoder();
        backRightModule.resetDriveEncoder();
    }

    /**
     * @param speeds The requested speeds of the chassis in m/s and rad/s
     */
    public void driveVelocity(ChassisSpeeds speeds) {
        drive(speeds, DriveMode.RawPower);
    }

    /**
     * @param speeds The requested speeds of the chassis from 1 to -1
     */
    public void drivePower(ChassisSpeeds speeds) {
        drive(new ChassisSpeeds(speeds.vxMetersPerSecond * Constants.driveMaxSpeed,
            speeds.vyMetersPerSecond * Constants.driveMaxSpeed, speeds.omegaRadiansPerSecond * Constants.driveMaxSpeed), DriveMode.RawPower);
    }

    /**
     * @param speeds The requested speeds of the chassis
     * @param maxSpeed The maximum possible value of the {@code ChassisSpeeds} object
     */
    private void drive(ChassisSpeeds speeds, DriveMode driveMode) {
        if(speeds.vxMetersPerSecond + speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond == 0) {
            setAllModuleDriveRawPower(0);
            selfTargetAllModuleAngles();
        } else {
            // Convert speeds to individual modules
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.driveMaxSpeed);

            for (int i = 0; i < states.length; i++) {
                // Optimize module rotation (instead of a >90 degree turn, turn less and flip wheel direction)
                states[i] = SwerveModuleState.optimize(states[i], swerveArray[i].getModuleRotation());

                swerveArray[i].setTurningTarget(states[i].angle);
                if (driveMode == DriveMode.Velocity)
                    swerveArray[i].setDriveVelocity(states[i].speedMetersPerSecond);
                else
                    swerveArray[i].setDrivePowerRaw(states[i].speedMetersPerSecond / Constants.driveMaxSpeed);
            }
        }
    }

    /**
     * @return Swerve module states in m/s
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < swerveArray.length; i++) {
            states[i] = swerveArray[i].getSwerveModuleState();
        }
        return states;
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

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    // TODO
    public ChassisSpeeds getFieldSpeeds() {
        return new ChassisSpeeds(0, 0, 0);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModulePosition[] getModulePositions() {
        return modulePositions;
    }

    public enum ModuleType { Neo, Falcon }

    public enum DriveMode { RawPower, Velocity }
}