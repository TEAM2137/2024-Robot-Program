package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PID;

public class FalconModule extends SwerveModule {

    public static class Constants {
        public static final double driveRatio = 1 / 6.75;
        public static final double measuredWheelDiameter = Units.inchesToMeters(4.0);

        // public static final double turningRatio = 1 / 21.42857;
        public static final double turningRatio = 21.42857;

        public static final boolean invertDriveMotor = true;
        public static final boolean invertTurningMotor = true;
        public static final boolean flipTurnDirection = true;

        public static final double driveMotorRamp = 0.0;

        public static double turningFeedForward = 0.75;
        public static PID turningPIDConstants = new PID(40, 0, 3);

        public static PID drivePIDConstants = new PID(10, 0, 0);
        public static SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.64728, 2.2607, 0.15911); //0.7, 2.15
    }

    private TalonFX driveMotor;
    private TalonFX turningMotor;
    private CANcoder encoder;

    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);
    private boolean reverseWheel;

    private double driveRawPower;
    private double driveVelocityTarget;
    private SimpleMotorFeedforward driveFeedForward;
    private DriveMode driveMode = DriveMode.RawPower;

    private TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration turningMotorConfig = new TalonFXConfiguration();

    private final MotionMagicVoltage turnPositionRequest = new MotionMagicVoltage(0);
    private final DutyCycleOut driveVoltageRequest = new DutyCycleOut(0);
    private final VelocityDutyCycle driveVelocityRequest = new VelocityDutyCycle(0);

    /**
     * Creats a swerve module
     * @param driveID CAN id of the drive motor
     * @param turningID CAN id of the turning motor
     * @param encoderID CAN id of the encoder
     * @param encoderOffset offset of the encoder on module rotation (wheels forward, bevels left)
     * @param moduleName name of the module for debug purposes
     */
    public FalconModule(int driveID, int turningID, int encoderID, double encoderOffset, String moduleName) {
        super(driveID, turningID, encoderID, encoderOffset, moduleName);

        // Drive motor setup
        this.driveMotor = new TalonFX(driveID, drivetrainCanBus);
        this.driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        this.driveMotor.setInverted(Constants.invertDriveMotor);

        driveMotorConfig.CurrentLimits = new CurrentLimitsConfigs() // New API doesn't have thresholds for stator limit. i hope this works
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentThreshold(60)
            .withSupplyTimeThreshold(1);
        
        driveMotorConfig.Feedback = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .withRotorToSensorRatio(Constants.driveRatio); // Again, params are missing. should have 10 second timeout. maybe it'll be fine?

        // Setup drive pid
        PID drivePIDConstants = Constants.drivePIDConstants;
        this.driveFeedForward = Constants.driveFeedforward;

        driveMotorConfig.Slot0.kP = drivePIDConstants.getP();
        driveMotorConfig.Slot0.kI = drivePIDConstants.getI();
        driveMotorConfig.Slot0.kD = drivePIDConstants.getD();
        
        this.driveMotor.getConfigurator().apply(driveMotorConfig);

        setDriveMode(true);

        // Turning motor setup
        this.turningMotor = new TalonFX(turningID, drivetrainCanBus);
        this.turningMotor.getConfigurator().apply(new TalonFXConfiguration());

        turningMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / Constants.turningRatio;
        turningMotorConfig.MotionMagic.MotionMagicAcceleration = turningMotorConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        turningMotorConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.turningRatio;
        turningMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;

        turningMotorConfig.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(20)
            .withSupplyCurrentThreshold(30)
            .withSupplyTimeThreshold(1);

        // turningMotorConfig.Feedback.RotorToSensorRatio = Constants.turningRatio;
        turningMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turningMotorConfig.Feedback.SensorToMechanismRatio = Constants.turningRatio;
        turningMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

        // Setup turning pid
        PID turningPIDConstants = Constants.turningPIDConstants;

        turningMotorConfig.Slot0.kP = turningPIDConstants.getP();
        turningMotorConfig.Slot0.kI = turningPIDConstants.getI();
        turningMotorConfig.Slot0.kD = turningPIDConstants.getD();
        
        this.turningMotor.getConfigurator().apply(turningMotorConfig);

        setTurnBrakeMode(true);

        // Encoder setup
        this.encoder = new CANcoder(encoderID, drivetrainCanBus);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = -encoderOffset;
        encoder.getConfigurator().apply(config);

        homeTurningMotor();

        this.selfTargetAngle();
    }

    /**
     * Creates a swerve module
     * @param constants the swerve module to create
     */
    public FalconModule(SwerveDrivetrain.Constants.SwerveModuleConstants constants) {
        this(constants.driveID, constants.turningID, constants.encoderID, constants.offset, constants.moduleName);
    }

    /**
     * Periodic loop of the subsystem
     */
    @Override
    public void periodic() {
        if(DriverStation.isDisabled()) selfTargetAngle();

        turningMotor.setControl(turnPositionRequest.withPosition(targetAngle.getRotations()));
        // turningMotor.set(0);

        switch(driveMode) {
            case RawPower: //for use in teleop
                driveMotor.setControl(driveVoltageRequest.withOutput(driveRawPower * (reverseWheel ? -1 : 1)));
                break;
            case Velocity: //for use in auto and autonomous trajectories
                driveMotor.setControl(driveVelocityRequest
                    .withVelocity(driveVelocityTarget / (Constants.measuredWheelDiameter * Math.PI))
                    .withFeedForward(driveFeedForward.calculate(driveVelocityTarget) / 12.0 * (reverseWheel ? -1 : 1))
                );
                break;
        }

        currentPosition = getModuleRotation().getDegrees();

        SmartDashboard.putNumber(moduleName + "-Position", currentPosition);
        SmartDashboard.putNumber(moduleName + "-Target", MathUtil.inputModulus(targetAngle.getDegrees(), -180, 180));
        SmartDashboard.putNumber(moduleName + "-CANCoder", encoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.updateValues();
    }

    /**
     * @return Rotation2d with the rotation of the module direct from encoder (not dealing with optimization)
     */
    @Override
    public Rotation2d getModuleRotation() {
        return Rotation2d.fromDegrees((turningMotor.getPosition().getValueAsDouble() * 360) % 360);
    }

    /**
     * @param target Rotation2d with the target angle, unoptimized
     */
    @Override
    public void setTurningTarget(Rotation2d target) {
        targetAngle = target;
    }

    @Override
    public void homeTurningMotor() {
        turningMotor.setPosition(encoder.getAbsolutePosition().getValueAsDouble() % 1);
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
        return driveMotor.getVelocity().getValueAsDouble() * Constants.driveRatio *
                (Math.PI * Constants.measuredWheelDiameter);
    }

    /**
     * @return The distance the wheel has driven
     */
    @Override
    public double getDriveDistance() {
        return driveMotor.getPosition().getValueAsDouble() * Constants.driveRatio *
                (Constants.measuredWheelDiameter * Math.PI);
    }

    /**
     * Resets drive encoder distance to zero.
     */
    @Override
    public void resetDriveEncoder() {
        driveMotor.setPosition(0, .01);
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
        driveMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setTurnBrakeMode(boolean brake) {
        turningMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void setupCANFrames() {
        // TODO: Figure out equivalents to these commented out CAN frames.

        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);

        driveMotor.getPosition().setUpdateFrequency(50);
        driveMotor.getSupplyCurrent().setUpdateFrequency(4);
        driveMotor.getStatorCurrent().setUpdateFrequency(4);
        driveMotor.getTorqueCurrent().setUpdateFrequency(10);
        driveMotor.getControlMode().setUpdateFrequency(4);
        driveMotor.getAcceleration().setUpdateFrequency(10);
        driveMotor.getAppliedRotorPolarity().setUpdateFrequency(10);

        // turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
        // turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        // turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        // turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
        // turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
        // turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
        // turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
        // turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
        // turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
        // turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
        // turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);

        turningMotor.getPosition().setUpdateFrequency(50);
        turningMotor.getSupplyCurrent().setUpdateFrequency(4);
        turningMotor.getStatorCurrent().setUpdateFrequency(4);
        turningMotor.getTorqueCurrent().setUpdateFrequency(10);
        turningMotor.getControlMode().setUpdateFrequency(4);
        turningMotor.getAcceleration().setUpdateFrequency(10);
        turningMotor.getAppliedRotorPolarity().setUpdateFrequency(10);
    }

    private enum DriveMode {
        RawPower, Velocity
    }
}