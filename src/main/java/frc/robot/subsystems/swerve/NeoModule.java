package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.PID;

public class NeoModule extends SubsystemBase {
    public static class Constants {
        public static final double driveRatio = 1 / 6.75;
        public static final double measuredWheelDiameter = Units.inchesToMeters(4.0);

        public static final double turningRatio = 1 / 12.8;

        public static final boolean invertDriveMotor = true;
        public static final boolean invertTurningMotor = false;
        //current limits

        public static final double driveMotorRamp = 0.0;

        public static double turningFeedForward = 0.75;
        public static PID turningPIDConstants = new PID(1, 0, 0.2);

        public static PID drivePIDConstants = new PID(3, 0, 0);
        public static SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.64728, 2.2607, 0.15911);
    }

    private CANSparkMax driveMotor;
    private CANSparkMax turningMotor;
    private CANcoder encoder;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turningEncoder;

    private SparkPIDController drivePID;
    private SparkPIDController turningPID;

    // private PIDController turningPID;

    private double encoderOffset;

    private Rotation2d turningSetpointRaw = Rotation2d.fromDegrees(0);
    // private Rotation2d turningSetpointCorrected = Rotation2d.fromDegrees(0);

    private double driveRawPower;
    private double driveVelocityTarget;
    // private PIDController drivePID;
    private SimpleMotorFeedforward driveFeedForward;
    private DriveMode driveMode = DriveMode.RawPower;

    public final String moduleName;

    /**
     * Creats a swerve module
     * @param driveID CAN id of the drive motor
     * @param turningID CAN id of the turning motor
     * @param encoderID CAN id of the encoder
     * @param encoderOffset offset of the encoder on module rotation (wheels forward, bevels left)
     * @param moduleName name of the module for debug purposes
     */
    public NeoModule(int driveID, int turningID, int encoderID, double encoderOffset, String moduleName) {
        // Drive motor setup
        this.driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        this.driveMotor.setInverted(Constants.invertDriveMotor);

        setDriveMode(true);

        // Turning motor setup
        this.turningMotor = new CANSparkMax(turningID, MotorType.kBrushless);
        this.turningMotor.setInverted(Constants.invertTurningMotor);

        setTurnBrakeMode(true);

        // Encoder setup
        this.encoder = new CANcoder(encoderID, SwerveDrivetrain.Constants.canBusName);
        //this.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180
        this.encoder.getConfigurator().apply(
            new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
            .withMagnetOffset(0));
        //this.encoder.configMagnetOffset(0);

        this.encoderOffset = encoderOffset;

        this.moduleName = moduleName;

        this.driveEncoder = driveMotor.getEncoder();
        this.driveEncoder.setPositionConversionFactor(Constants.driveRatio * Math.PI * Constants.measuredWheelDiameter);
        this.driveEncoder.setVelocityConversionFactor(Constants.driveRatio / 60 * Math.PI * Constants.measuredWheelDiameter);
        this.turningEncoder = turningMotor.getEncoder();
        this.turningEncoder.setPositionConversionFactor(Constants.turningRatio * 360);

        // Setup turning pid
        this.turningPID = this.turningMotor.getPIDController();
        this.turningPID.setFeedbackDevice(turningEncoder);

        PID turningPIDConstants = Constants.turningPIDConstants;
        this.turningPID.setFF(0);
        this.turningPID.setP(turningPIDConstants.getP());
        this.turningPID.setI(turningPIDConstants.getI());
        this.turningPID.setD(turningPIDConstants.getD());

        this.turningPID.setPositionPIDWrappingMinInput(0);
        this.turningPID.setPositionPIDWrappingMaxInput(360);
        this.turningPID.setPositionPIDWrappingEnabled(true);

        // Setup drive pid
        this.drivePID = this.driveMotor.getPIDController();
        this.drivePID.setFeedbackDevice(driveEncoder);

        PID drivePIDConstants = Constants.drivePIDConstants;
        this.drivePID.setFF(0);
        this.drivePID.setP(drivePIDConstants.getP());
        this.drivePID.setI(drivePIDConstants.getI());
        this.drivePID.setD(drivePIDConstants.getD());
        this.driveFeedForward = Constants.driveFeedforward;

        selfTargetAngle();
        homeTurningMotor();
    }

    /**
     * Creates a swerve module
     * @param constants the swerve module to create
     */
    public NeoModule(SwerveDrivetrain.Constants.SwerveModuleConstants constants) {
        this(constants.driveID, constants.turningID, constants.encoderID, constants.offset, constants.moduleName);
    }

    /**
     * Periodic loop of the subsystem
     */
    @Override
    public void periodic() {
        // if robot is disabled, target modules to their current angle
        // if you're doing some types of debugging, disable this
        if(DriverStation.isDisabled()) {
            selfTargetAngle();
        }

        double targetDegrees = turningSetpointRaw.getDegrees();
        double currentDegrees = getModuleRotation().getDegrees() + encoderOffset;
        boolean stopTurn = Math.abs(targetDegrees - currentDegrees) < 2 || Math.abs(targetDegrees + 360 - currentDegrees) < 2 || Math.abs(targetDegrees - 360 - currentDegrees) < 2;;

        if (!RobotContainer.disableSwerve) {
            if (stopTurn) {
                turningMotor.set(0);
            } else {
                turningPID.setReference(targetDegrees, ControlType.kPosition, 0, 
                        Constants.turningFeedForward * ((targetDegrees - currentDegrees > 0) ? 1 : -1), ArbFFUnits.kPercentOut);
            }

            switch(driveMode) {
                case RawPower: //for use in teleop
                    driveMotor.set(driveRawPower);
                    break;
                case Velocity: //for use in auto and autonomous trajectories
                    drivePID.setReference(driveVelocityTarget, ControlType.kVelocity, 0, driveFeedForward.calculate(driveVelocityTarget), ArbFFUnits.kPercentOut);
                    break;
            }
        }

        SmartDashboard.putNumber("Module-" + moduleName + "-Heading Position", currentDegrees - encoderOffset);
        SmartDashboard.putNumber("Module-" + moduleName + "-Heading Target", targetDegrees);
        // SmartDashboard.putBoolean("Module-" + moduleName + "-Stop Rotation?", stopTurn);
        // SmartDashboard.putNumber("Module-" + moduleName + "-Heading RAW", encoder.getAbsolutePosition().getValueAsDouble() + encoderOffset);
        // SmartDashboard.putNumber("" + moduleName + "/Heading Error", turningPID.getPositionError());
        //SmartDashboard.putNumber("Module-" + moduleName + "-Heading Power", turningMotor.getAppliedOutput());

        SmartDashboard.putNumber("Module-" + moduleName + "-Drive Power", driveMotor.getAppliedOutput());
        //SmartDashboard.putNumber("Module-" + moduleName + "-Velocity Target", Math.abs(driveVelocityTarget));
        //SmartDashboard.putNumber("Module-" + moduleName + "-Velocity", Math.abs(getDriveVelocity()));
    }

    /**
     * Do not call this method often, as it negates the entire purpose of reducing CAN frames
     * @return Rotation2d with the rotation of the module direct from encoder (not dealing with optimization)
     */
    public Rotation2d getModuleRotation() {
        return Rotation2d.fromDegrees(turningEncoder.getPosition() % 360);
    }

    /**
     * @param target Rotation2d with the target angle, unoptimized
     */
    public void setTurningTarget(Rotation2d target) {
        turningSetpointRaw = target;
    }

    public void homeTurningMotor() {
        turningEncoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * @param power from -1 to 1
     */
    public void setDrivePowerRaw(double power) {
        driveRawPower = power;
        driveMode = DriveMode.RawPower;
    }


    /**
     * @param velocity in meters per second
     */
    public void setDriveVelocity(double velocity) {
        driveVelocityTarget = velocity;
        driveMode = DriveMode.Velocity;
    }

    /**
     * @return Drive wheel velocity in meters per second
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * @return The distance the wheel has driven
     */
    public double getDriveDistance() {
        return driveEncoder.getPosition();
    }

    /**
     * Resets drive encoder distance to zero.
     */
    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getDriveVelocity(), getModuleRotation());
    }

    public void selfTargetAngle() {
//        setTurningTarget(getModuleRotation());
    }

    public void setDriveMode(boolean brake) {
        driveMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void setTurnBrakeMode(boolean brake) {
        turningMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    private enum DriveMode {
        RawPower, Velocity
    }
}