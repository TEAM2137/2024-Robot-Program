package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;
import frc.robot.util.PID;
 
public class ShooterSubsystem extends SubsystemBase {

    public static class Constants {
        public static PID shooterPID = new PID(0.0006, 0, 0);
        public static SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0.65, 2.2, 0.15);

        public static double longAngle = 44;
        public static double shortAngle = 15;
    }

    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;
    private CANSparkMax pivotMotor;

    private SparkPIDController topPID;
    private SparkPIDController bottomPID;

    private RelativeEncoder topEncoder;

    //private SparkPIDController pivotPID;

    private AbsoluteEncoder absolutePivotEncoder;
    private double pivotTarget;
    private boolean running;

    private double targetRPM;

    public ShooterSubsystem() {
        super();

        pivotTarget = Constants.longAngle;

        topMotor = new CANSparkMax(CanIDs.get("shooter-top"), MotorType.kBrushless);
        bottomMotor = new CANSparkMax(CanIDs.get("shooter-bottom"), MotorType.kBrushless);
        topMotor.stopMotor();
        bottomMotor.stopMotor();

        topPID = topMotor.getPIDController();
        topEncoder = topMotor.getEncoder();
        topPID.setFeedbackDevice(topMotor.getEncoder());
        topPID.setP(Constants.shooterPID.getP());
        topPID.setI(Constants.shooterPID.getI());
        topPID.setD(Constants.shooterPID.getD());
        topPID.setFF(0);

        bottomPID = bottomMotor.getPIDController();
        bottomPID.setFeedbackDevice(bottomMotor.getEncoder());
        bottomPID.setP(Constants.shooterPID.getP());
        bottomPID.setI(Constants.shooterPID.getI());
        bottomPID.setD(Constants.shooterPID.getD());
        bottomPID.setFF(0);

        pivotMotor = new CANSparkMax(CanIDs.get("shooter-pivot"), MotorType.kBrushless);
        pivotMotor.stopMotor();
        pivotMotor.setIdleMode(IdleMode.kBrake);

        absolutePivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        absolutePivotEncoder.setPositionConversionFactor(360);
        absolutePivotEncoder.setZeroOffset(259.89978027);

        /* The PID is removed as of now because it's trash and it rips the shooter apart :( */

        //pivotPID = pivotMotor.getPIDController();
        //pivotPID.setFeedbackDevice(absolutePivotEncoder);
        //pivotPID.setP(Constants.pivotPID.getP());
        //pivotPID.setI(Constants.pivotPID.getI());
        //pivotPID.setD(Constants.pivotPID.getD());
        //pivotPID.setFF(Constants.pivotPID.getFF());
    }

    /**
     * Stops the shooter if it's running and starts it
     * if it's not, at the specified speed
     * @param speed the RPM to set the wheels to
     * @return the command
     */
    public Command toggleShooter(double speed) {
        return runOnce(() -> {
            if (!running) CommandScheduler.getInstance().schedule(startShooter(speed));
            else CommandScheduler.getInstance().schedule(stopShooter());
        });
    }

    /**
     * Starts the shooter wheels
     * @param speed the RPM to set the wheels to
     * @return the command
     */
    public Command startShooter(double speed) {
        return runOnce(() -> {
            topPID.setReference(speed, ControlType.kVelocity, 0,
                Constants.shooterFF.calculate(speed), ArbFFUnits.kPercentOut);
            bottomPID.setReference(speed, ControlType.kVelocity, 0,
                Constants.shooterFF.calculate(speed), ArbFFUnits.kPercentOut);
            targetRPM = speed;
        }).andThen(() -> running = true);
    }

    /**
     * Stops the shooter wheel motors
     * @return the command
     */
    public Command stopShooter() {
        return runOnce(() -> {
            topPID.setReference(0, ControlType.kVelocity);
            bottomPID.setReference(0, ControlType.kVelocity);
            targetRPM = 0;
        }).andThen(() -> running = false);
    }

    /**
     * @return true if the shooter wheels are running
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Set the position of the shooter pivot
     * @param target The target angle in degrees
     * @return The command that moves the pivot
     */
    public Command setPivotTarget(double target) {
        return runOnce(() -> pivotTarget = target);
    }

    /**
     * Non-command version of setPivotTarget()
     * @param target The target angle in degrees
     */
    public void setPivotTargetRaw(double target) {
        pivotTarget = target;
    }

    /**
     * @return the current pivot target
     */
    public double getPivotTarget() {
        return pivotTarget;
    }

    /**
     * Sets the pivot to target its home angle of 0 degrees
     * @return the command
     */
    public Command stowPivot() {
        return setPivotTarget(0);
    }

    @Override
    public void periodic() {
        super.periodic();

        // Calculate the target and current rotations
        double encoderPos = absolutePivotEncoder.getPosition();
        Rotation2d target = Rotation2d.fromDegrees(pivotTarget);
        Rotation2d current = Rotation2d.fromDegrees(encoderPos);

        double error = Math.max(Math.min(target.minus(current).getDegrees() / 180.0,
            /* Max motor speed */ 0.12), /* Min motor speed */ -0.12);
        
        pivotMotor.set(error - 0.005);

        // pivotPID.setReference(pivotTarget, ControlType.kPosition);

        // Display values
        SmartDashboard.putNumber("Shooter RPM", topEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Target", targetRPM);
        SmartDashboard.putNumber("Shooter Pivot Encoder Position", absolutePivotEncoder.getPosition());
        SmartDashboard.putNumber("Shooter Pivot Encoder Target", pivotTarget);
        SmartDashboard.updateValues();
    }
}