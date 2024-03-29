package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
import frc.robot.util.PID;

public class NeoModule extends SwerveModule {
    public static class Constants {
        public static final double driveRatio = 1 / 6.75;
        public static final double measuredWheelDiameter = Units.inchesToMeters(4.0);

        public static final double turningRatio = 1 / 12.8;

        public static final boolean invertDriveMotor = false;
        public static final boolean invertTurningMotor = false;
        //current limits

        public static final double driveMotorRamp = 0.0;

        public static double turningFeedForward = 0.75;
        public static PID turningPIDConstants = new PID(40, 0, 100, 0.2);

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

    private Rotation2d turningSetpointRaw = Rotation2d.fromDegrees(0);

    private double driveRawPower;
    private double driveVelocityTarget;
    private SimpleMotorFeedforward driveFeedForward;
    private DriveMode driveMode = DriveMode.RawPower;

    /**
     * Creats a swerve module
     * @param driveID CAN id of the drive motor
     * @param turningID CAN id of the turning motor
     * @param encoderID CAN id of the encoder
     * @param encoderOffset offset of the encoder on module rotation (wheels forward, bevels left)
     * @param moduleName name of the module for debug purposes
     */
    public NeoModule(int driveID, int turningID, int encoderID, double encoderOffset, String moduleName) {
        super(driveID, turningID, encoderID, encoderOffset, moduleName);

        // Drive motor setup
        this.driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        this.driveMotor.setInverted(Constants.invertDriveMotor);

        setDriveMode(true);

        // Turning motor setup
        this.turningMotor = new CANSparkMax(turningID, MotorType.kBrushless);
        this.turningMotor.setInverted(Constants.invertTurningMotor);

        setTurnBrakeMode(true);

        // Encoder setup
        this.encoder = new CANcoder(encoderID, rioCanBus);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = -encoderOffset;
        encoder.getConfigurator().apply(config);

        this.driveEncoder = driveMotor.getEncoder();
        this.driveEncoder.setPosition(0);
        this.driveEncoder.setPositionConversionFactor(Constants.driveRatio * Math.PI * Constants.measuredWheelDiameter);
        this.driveEncoder.setVelocityConversionFactor(Constants.driveRatio / 60 * Math.PI * Constants.measuredWheelDiameter);
        this.turningEncoder = turningMotor.getEncoder();
        this.turningEncoder.setPositionConversionFactor(Constants.turningRatio * 360);
        
        // Setup turning pid
        this.turningPID = this.turningMotor.getPIDController();
        this.turningPID.setFeedbackDevice(turningEncoder);

        PID turningPIDConstants = Constants.turningPIDConstants;
        //this.turningPID.setFF(turningPIDConstants.getFF());
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
        double targetDegrees = turningSetpointRaw.getDegrees();
        double currentDegrees = getModuleRotation().getDegrees();

        boolean stopTurn = 
            Math.abs(targetDegrees - currentDegrees) < 2 || 
            Math.abs(targetDegrees + 360 - currentDegrees) < 2 || 
            Math.abs(targetDegrees - 360 - currentDegrees) < 2;

        if(DriverStation.isDisabled()) {
            selfTargetAngle();
        }

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

        currentPosition = currentDegrees;

        SmartDashboard.putNumber(moduleName + "-Position", currentDegrees);
        SmartDashboard.putNumber(moduleName + "-Target", targetDegrees);
        SmartDashboard.putNumber(moduleName + "-CANCoder", encoder.getAbsolutePosition().getValueAsDouble());
        //SmartDashboard.putNumber("Module-" + moduleName + "-Drive Power", driveMotor.getAppliedOutput());
    }

    /**
     * Do not call this method often, as it negates the entire purpose of reducing CAN frames
     * @return Rotation2d with the rotation of the module direct from encoder (not dealing with optimization)
     */
    @Override
    public Rotation2d getModuleRotation() {
        return Rotation2d.fromDegrees((turningEncoder.getPosition() % 360));
    }

    /**
     * @param target Rotation2d with the target angle, unoptimized
     */
    @Override
    public void setTurningTarget(Rotation2d target) {
        turningSetpointRaw = target;
    }

    @Override
    public void homeTurningMotor() {
        turningEncoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    /**
     * @param power from -1 to 1
     */
    @Override
    public void setDrivePowerRaw(double power) {
        driveRawPower = power;
        driveMode = DriveMode.RawPower;
    }


    /**
     * @param velocity in meters per second
     */
    @Override
    public void setDriveVelocity(double velocity) {
        driveVelocityTarget = velocity;
        driveMode = DriveMode.Velocity;
    }

    /**
     * @return Drive wheel velocity in meters per second
     */
    @Override
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * @return The distance the wheel has driven
     */
    @Override
    public double getDriveDistance() {
        return driveEncoder.getPosition();
    }

    /**
     * Resets drive encoder distance to zero.
     */
    @Override
    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    @Override
    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getDriveVelocity(), getModuleRotation());
    }

    @Override
    public void selfTargetAngle() {
//        setTurningTarget(getModuleRotation());
    }

    @Override
    public void setDriveMode(boolean brake) {
        driveMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setTurnBrakeMode(boolean brake) {
        turningMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    private enum DriveMode {
        RawPower, Velocity
    }
}