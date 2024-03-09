package frc.robot.subsystems;

import java.util.Random;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;
 
public class ShooterSubsystem extends SubsystemBase {

    public static class Constants {
        public static double maxAngle = 164.4;
        public static double midAngle = 142.8;
        public static double ampAngle = 134.0; // TODO tune this
        public static double stowAngle = 107.9;
        public static double minAngle = 107.9;
    }

    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;
    private CANSparkMax pivotMotor;

    private AbsoluteEncoder pivotEncoder;
    private double pivotTarget;
    private boolean running;

    public ShooterSubsystem() {
        super();

        pivotTarget = Constants.minAngle;

        topMotor = new CANSparkMax(CanIDs.get("shooter-top"), MotorType.kBrushless);
        topMotor.setIdleMode(IdleMode.kCoast);
        topMotor.stopMotor();
        
        bottomMotor = new CANSparkMax(CanIDs.get("shooter-bottom"), MotorType.kBrushless);
        bottomMotor.setIdleMode(IdleMode.kCoast);
        bottomMotor.stopMotor();

        pivotMotor = new CANSparkMax(CanIDs.get("shooter-pivot"), MotorType.kBrushless);
        pivotMotor.stopMotor();
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pivotEncoder.setPositionConversionFactor(360);
        pivotEncoder.setZeroOffset(100);
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

    public Command startAndRun(double speed, double time) {
        return run(() -> setPowerRaw(speed)).withTimeout(time);
    }

    /**
     * Starts the shooter wheels
     * @param speed the RPM to set the wheels to
     * @return the command
     */
    public Command startShooter(double speed) {
        return runOnce(() -> {
            bottomMotor.set(speed);
            topMotor.set(speed);
        }).andThen(() -> running = true);
    }

    /**
     * Stops the shooter wheel motors
     * @return the command
     */
    public Command stopShooter() {
        return runOnce(() -> {
            bottomMotor.set(0);
            topMotor.set(0);
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
     * Sets the pivot to target its min angle
     * @return the command
     */
    public Command stowPivot() {
        return setPivotTarget(Constants.minAngle);
    }

    @Override
    public void periodic() {
        super.periodic();

        // Calculate the target and current rotations
        double encoderPos = pivotEncoder.getPosition();
        Rotation2d target = Rotation2d.fromDegrees(pivotTarget);
        Rotation2d current = Rotation2d.fromDegrees(encoderPos);

        double error = Math.max(Math.min(target.minus(current).getDegrees() / 180.0,
            /* Max motor speed */ 0.12), /* Min motor speed */ -0.12);
        
        pivotMotor.set(error - 0.005);

        // Display values
        SmartDashboard.putNumber("Shooter Position", pivotEncoder.getPosition());
        SmartDashboard.updateValues();
    }

    public Command setRandomPivotTarget() {
        return runOnce(() -> setPivotTargetRaw(new Random().nextInt(32, 50)));
    }

    public void setPowerRaw(double speed) {
        bottomMotor.set(speed);
        topMotor.set(speed);
    }
}