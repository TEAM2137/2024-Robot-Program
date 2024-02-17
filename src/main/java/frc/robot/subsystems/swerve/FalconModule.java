package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.DriverStation;
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

        public static final double turningRatio = 1 / 12.8;

        public static final boolean invertDriveMotor = true;
        public static final boolean invertTurningMotor = false;
        // public static final SupplyCurrentLimitConfiguration driveMotorSupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 60 ,1);
        // public static final StatorCurrentLimitConfiguration driveMotorStatorCurrentLimit = new StatorCurrentLimitConfiguration(true, 40, 60, 1);
        // public static final SupplyCurrentLimitConfiguration turningMotorSupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 20, 30, 1);
        // public static final StatorCurrentLimitConfiguration turningMotorStatorCurrentLimit = new StatorCurrentLimitConfiguration(true, 20, 30, 1);

        public static final double driveMotorRamp = 0.0;

        public static double turningFeedForward = 0.75; //0.8
        //        public static PID turningPIDConstants = new PID(0.21, 0, 0.0015); // in the air
//        public staticPID turningPIDConstants = new PID(0.1, 0, -0.0000000000000000000000001); // carpet
        public static PID turningPIDConstants = new PID(0.2, 0, 0.1); // carpet

        public static PID drivePIDConstants = new PID(0.10237, 0, 0);// 0.1
        public static SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.64728, 2.2607, 0.15911); //0.7, 2.15
    }

    private TalonFX driveMotor;
    private TalonFX turningMotor;
    private CANcoder encoder;

    // private PIDController turningPID;

    // private double encoderOffset;

    private Rotation2d turningSetpointRaw = Rotation2d.fromDegrees(0);
    // private Rotation2d turningSetpointCorrected = Rotation2d.fromDegrees(0);
    private boolean reverseWheel;

    private double driveRawPower;
    private double driveVelocityTarget;
    // private PIDController drivePID;
    private SimpleMotorFeedforward driveFeedForward;
    private DriveMode driveMode = DriveMode.RawPower;

    // public final String moduleName;

    private TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration turningMotorConfig = new TalonFXConfiguration();

    private final PositionDutyCycle turningPositionRequest = new PositionDutyCycle(0);
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
        this.driveMotor = new TalonFX(driveID, canBusName);
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
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor); // Again, params are missing. should have 10 second timeout. maybe it'll be fine?

        // this.driveMotor.configSupplyCurrentLimit(Constants.driveMotorSupplyCurrentLimit);
        // this.driveMotor.configStatorCurrentLimit(Constants.driveMotorStatorCurrentLimit);
//        this.driveMotor.configClosedloopRamp(Constants.driveMotorRamp);
//        this.driveMotor.configOpenloopRamp(Constants.driveMotorRamp);
        // this.driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

        // Setup drive pid
        PID drivePIDConstants = Constants.drivePIDConstants;
        this.driveFeedForward = Constants.driveFeedforward;

        driveMotorConfig.Slot0.kP = drivePIDConstants.getP();
        driveMotorConfig.Slot0.kI = drivePIDConstants.getI();
        driveMotorConfig.Slot0.kD = drivePIDConstants.getD();
        
        this.driveMotor.getConfigurator().apply(driveMotorConfig);

        setDriveMode(true);

        // Turning motor setup
        this.turningMotor = new TalonFX(turningID, canBusName);
        this.turningMotor.getConfigurator().apply(new TalonFXConfiguration());
        this.turningMotor.setInverted(Constants.invertTurningMotor);
        // this.turningMotor.configSupplyCurrentLimit(Constants.turningMotorSupplyCurrentLimit);
        // this.turningMotor.configStatorCurrentLimit(Constants.turningMotorStatorCurrentLimit);

        turningMotorConfig.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(20)
            .withSupplyCurrentThreshold(30)
            .withSupplyTimeThreshold(1);

        turningMotorConfig.Feedback = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        // Setup turning pid
        PID turningPIDConstants = Constants.turningPIDConstants;

        turningMotorConfig.Slot0.kP = turningPIDConstants.getP();
        turningMotorConfig.Slot0.kI = turningPIDConstants.getI();
        turningMotorConfig.Slot0.kD = turningPIDConstants.getD();
        
        this.turningMotor.getConfigurator().apply(turningMotorConfig);

        setTurnBrakeMode(true);

        // Encoder setup
        this.encoder = new CANcoder(encoderID, canBusName);
        // this.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180
        // this.encoder.configMagnetOffset(0);
        this.encoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(-encoderOffset));

        homeTurningMotor();

        this.selfTargetAngle();

        setupCANFrames();
    }

    /**
     * Creates a swerve module
     * @param constants the swerve module to create
     */
    public FalconModule(SwerveDrivetrain.Constants.SwerveModuleConstants constants) {
        super(constants);
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

        // optimize the wheel angle to avoid >360 turns (by adding/subtracting 180)
        double targetDegrees = turningSetpointRaw.getDegrees();
//        double delta = (getModuleRotation().getDegrees() % 360) - targetDegrees;
//        if (delta > 180) {
//            targetDegrees += 360;
//
//        } else if (delta < 180) {
//            targetDegrees -= 360;
//        }
//
//        delta = (getModuleRotation().getDegrees() % 360) - targetDegrees;
//        if (delta > 90 || delta < -90) {
//            if(delta > 90){
//                targetDegrees += 180;
//            }
//            else if(delta < -90){
//                targetDegrees -= 180;
//            }
//            reverseWheel = true;
//        } else{
//            reverseWheel = false;
//        }

//        targetDegrees = 0;

        // target at wheel -> rotations at wheel -> rotations at motor -> counts at motor
        // turningMotor.set(ControlMode.Position, targetDegrees / 360.0 / Constants.turningRatio * 2048);
        turningMotor.setControl(turningPositionRequest.withPosition(targetDegrees / 360.0 / Constants.turningRatio * 2048));

        switch(driveMode) {
            case RawPower: //for use in teleop
                // driveMotor.set(ControlMode.PercentOutput, driveRawPower * (reverseWheel ? -1 : 1));
                driveMotor.setControl(driveVoltageRequest.withOutput(driveRawPower * (reverseWheel ? -1 : 1)));
                break;
            case Velocity: //for use in auto and autonomous trajectories
                // m/s -> rotations@wheel/second -> rotations@motor/second -> ticks@motor/second -> ticks@motor/100ms
                driveMotor.setControl(driveVelocityRequest
                    .withVelocity(driveVelocityTarget / (Constants.measuredWheelDiameter * Math.PI))
                    .withFeedForward(driveFeedForward.calculate(driveVelocityTarget) / 12.0 * (reverseWheel ? -1 : 1))
                );
                // driveMotor.set(ControlMode.Velocity, driveVelocityTarget / (Constants.measuredWheelDiameter * Math.PI)
                //         / Constants.driveRatio * 2048 / 10 * (reverseWheel ? -1 : 1),
                //         DemandType.ArbitraryFeedForward, driveFeedForward.calculate(driveVelocityTarget) / 12.0 * (reverseWheel ? -1 : 1));
                break;
        }

        currentPosition = getModuleRotation().getDegrees();

        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Heading Position", getModuleRotation().getDegrees());
        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Heading RAW", encoder.getAbsolutePosition().getValueAsDouble() + encoderOffset);
        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Heading Target", targetDegrees);
//        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Heading Error", turningPID.getPositionError());
        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Heading Power", turningMotor.getMotorVoltage().getValueAsDouble()); // Not 1:1 with previous. this is now in percent

        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Drive Power", driveMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Velocity Target", Math.abs(driveVelocityTarget));
        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Velocity", Math.abs(getDriveVelocity()));
    }

    /**
     * Do not call this method often, as it negates the entire purpose of reducing CAN frames
     * @return Rotation2d with the rotation of the module direct from encoder (not dealing with optimization)
     */
    @Override
    public Rotation2d getModuleRotation() {
        // ticks @ motor -> rotations @ motor -> rotations @ turret -> degrees @ turret
        return Rotation2d.fromDegrees((turningMotor.getPosition().getValueAsDouble() / 2048.0 * Constants.turningRatio * 360) % 360);
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
        // degrees @ turret -> rotations @ turret -> rotations @ motor -> ticks @ motor
        turningMotor.setPosition((((encoder.getAbsolutePosition().getValueAsDouble() + encoderOffset + 180) % 360) - 180) / 360.0 / Constants.turningRatio * 2048);
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
        // counts/100ms@motor -> counts/s @ motor -> rotations/s @ motor -> rotations/s @ wheel -> meters/s @ wheel
        return driveMotor.getVelocity().getValueAsDouble() * 10 / 2048 * Constants.driveRatio *
                (Math.PI * Constants.measuredWheelDiameter);
    }

    /**
     * @return The distance the wheel has driven
     */
    @Override
    public double getDriveDistance() {
        // counts @ motor -> rotations @ motor -> rotations @ wheel -> distance meters
        return driveMotor.getPosition().getValueAsDouble() / 2048.0 * Constants.driveRatio *
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
        driveMotor.getSupplyCurrent().setUpdateFrequency(10);
        driveMotor.getStatorCurrent().setUpdateFrequency(10);
        driveMotor.getTorqueCurrent().setUpdateFrequency(10);
        driveMotor.getControlMode().setUpdateFrequency(4);

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
        turningMotor.getSupplyCurrent().setUpdateFrequency(10);
        turningMotor.getStatorCurrent().setUpdateFrequency(10);
        turningMotor.getTorqueCurrent().setUpdateFrequency(10);
        turningMotor.getControlMode().setUpdateFrequency(4);
    }

    private enum DriveMode {
        RawPower, Velocity
    }
}