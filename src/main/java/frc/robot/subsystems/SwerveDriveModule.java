package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PID;

public class SwerveDriveModule extends SubsystemBase {

    public static class Constants {
        public static final double driveRatio = 1 / 6.75;
        public static final double measuredWheelDiameter = Units.inchesToMeters(4.0);

        public static final double turningRatio = 1 / 12.8;

        public static final boolean invertDriveMotor = true;
        public static final boolean invertTurningMotor = false;
        public static final SupplyCurrentLimitConfiguration driveMotorSupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 60 ,1);
        public static final StatorCurrentLimitConfiguration driveMotorStatorCurrentLimit = new StatorCurrentLimitConfiguration(true, 40, 60, 1);
        public static final SupplyCurrentLimitConfiguration turningMotorSupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 20, 30, 1);
        public static final StatorCurrentLimitConfiguration turningMotorStatorCurrentLimit = new StatorCurrentLimitConfiguration(true, 20, 30, 1);

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
    private CANCoder encoder;

    // private PIDController turningPID;

    private double encoderOffset;

    private Rotation2d turningSetpointRaw = Rotation2d.fromDegrees(0);
    // private Rotation2d turningSetpointCorrected = Rotation2d.fromDegrees(0);
    private boolean reverseWheel;

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
    public SwerveDriveModule(int driveID, int turningID, int encoderID, double encoderOffset, String moduleName) {
        // Drive motor setup
        this.driveMotor = new TalonFX(driveID, SwerveDrivetrain.Constants.canBusName);
        this.driveMotor.configFactoryDefault();
        this.driveMotor.setInverted(Constants.invertDriveMotor);
        this.driveMotor.configSupplyCurrentLimit(Constants.driveMotorSupplyCurrentLimit);
        this.driveMotor.configStatorCurrentLimit(Constants.driveMotorStatorCurrentLimit);
//        this.driveMotor.configClosedloopRamp(Constants.driveMotorRamp);
//        this.driveMotor.configOpenloopRamp(Constants.driveMotorRamp);
        this.driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

        setDriveMode(true);

        // Turning motor setup
        this.turningMotor = new TalonFX(turningID, SwerveDrivetrain.Constants.canBusName);
        this.turningMotor.configFactoryDefault();
        this.turningMotor.setInverted(Constants.invertTurningMotor);
        this.turningMotor.configSupplyCurrentLimit(Constants.turningMotorSupplyCurrentLimit);
        this.turningMotor.configStatorCurrentLimit(Constants.turningMotorStatorCurrentLimit);

        setTurnBrakeMode(true);

        // Encoder setup
        this.encoder = new CANCoder(encoderID, SwerveDrivetrain.Constants.canBusName);
        this.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180
        this.encoder.configMagnetOffset(0);

        this.encoderOffset = encoderOffset;

        this.moduleName = moduleName;

        homeTurningMotor();

        // Setup turning pid
        PID turningPIDConstants = Constants.turningPIDConstants;
        this.turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        this.turningMotor.config_kP(0, turningPIDConstants.getP());
        this.turningMotor.config_kI(0, turningPIDConstants.getI());
        this.turningMotor.config_kD(0, turningPIDConstants.getD());

        // Setup drive pid
        PID drivePIDConstants = Constants.drivePIDConstants;
        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        this.driveMotor.config_kP(0, drivePIDConstants.getP());
        this.driveMotor.config_kI(0, drivePIDConstants.getI());
        this.driveMotor.config_kD(0, drivePIDConstants.getD());
        this.driveFeedForward = Constants.driveFeedforward;

        this.selfTargetAngle();

        setupCANFrames();
    }

    /**
     * Creates a swerve module
     * @param constants the swerve module to create
     */
    public SwerveDriveModule(SwerveDrivetrain.Constants.SwerveModuleConstants constants) {
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
        turningMotor.set(ControlMode.Position, targetDegrees / 360.0 / Constants.turningRatio * 2048);

        switch(driveMode) {
            case RawPower: //for use in teleop
                driveMotor.set(ControlMode.PercentOutput, driveRawPower * (reverseWheel ? -1 : 1));
                break;
            case Velocity: //for use in auto and autonomous trajectories
                // m/s -> rotations@wheel/second -> rotations@motor/second -> ticks@motor/second -> ticks@motor/100ms
                driveMotor.set(ControlMode.Velocity, driveVelocityTarget / (Constants.measuredWheelDiameter * Math.PI)
                        / Constants.driveRatio * 2048 / 10 * (reverseWheel ? -1 : 1),
                        DemandType.ArbitraryFeedForward, driveFeedForward.calculate(driveVelocityTarget) / 12.0 * (reverseWheel ? -1 : 1));
                break;
        }

        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Heading Position", getModuleRotation().getDegrees());
        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Heading RAW", encoder.getAbsolutePosition() + encoderOffset);
        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Heading Target", targetDegrees);
//        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Heading Error", turningPID.getPositionError());
        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Heading Power", turningMotor.getMotorOutputPercent());

        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Drive Power", driveMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Velocity Target", Math.abs(driveVelocityTarget));
        SmartDashboard.putNumber("drivetrain/" + moduleName + "/Velocity", Math.abs(getDriveVelocity()));
    }

    /**
     * Do not call this method often, as it negates the entire purpose of reducing CAN frames
     * @return Rotation2d with the rotation of the module direct from encoder (not dealing with optimization)
     */
    public Rotation2d getModuleRotation() {
        // ticks @ motor -> rotations @ motor -> rotations @ turret -> degrees @ turret
        return Rotation2d.fromDegrees((turningMotor.getSelectedSensorPosition() / 2048.0 * Constants.turningRatio * 360) % 360);
    }

    /**
     * @param target Rotation2d with the target angle, unoptimized
     */
    public void setTurningTarget(Rotation2d target) {
        turningSetpointRaw = target;
    }

    public void homeTurningMotor() {
        // degrees @ turret -> rotations @ turret -> rotations @ motor -> ticks @ motor
        turningMotor.setSelectedSensorPosition((((encoder.getAbsolutePosition() + encoderOffset + 180) % 360) - 180) / 360.0 / Constants.turningRatio * 2048);
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
        // counts/100ms@motor -> counts/s @ motor -> rotations/s @ motor -> rotations/s @ wheel -> meters/s @ wheel
        return driveMotor.getSensorCollection().getIntegratedSensorVelocity() * 10 / 2048 * Constants.driveRatio *
                (Math.PI * Constants.measuredWheelDiameter);
    }

    /**
     * @return The distance the wheel has driven
     */
    public double getDriveDistance() {
        // counts @ motor -> rotations @ motor -> rotations @ wheel -> distance meters
        return driveMotor.getSensorCollection().getIntegratedSensorPosition() / 2048.0 * Constants.driveRatio *
                (Constants.measuredWheelDiameter * Math.PI);
    }

    /**
     * Resets drive encoder distance to zero.
     */
    public void resetDriveEncoder() {
        driveMotor.getSensorCollection().setIntegratedSensorPosition(0, 10);
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getDriveVelocity(), getModuleRotation());
    }

    public void selfTargetAngle() {
//        setTurningTarget(getModuleRotation());
    }

    public void setDriveMode(boolean brake) {
        driveMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setTurnBrakeMode(boolean brake) {
        turningMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setupCANFrames() {
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);


        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);
    }

    private enum DriveMode {
        RawPower, Velocity
    }
}