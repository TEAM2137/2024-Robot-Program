package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;
 
public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;

    private CANSparkMax pivotMotor;

    private AbsoluteEncoder absolutePivotEncoder;

    private SparkPIDController pivotPID;

    private double pivotTarget = 22;

    public ShooterSubsystem() {
        super();
        topMotor = new CANSparkMax(CanIDs.get("shooter-top"), MotorType.kBrushless);
        bottomMotor = new CANSparkMax(CanIDs.get("shooter-bottom"), MotorType.kBrushless);

        //topPID = topMotor.getPIDController();
        //bottomPID = bottomMotor.getPIDController();
        //topPID.setFeedbackDevice(topEncoder);
        //bottomPID.setFeedbackDevice(bottomEncoder);

        pivotMotor = new CANSparkMax(CanIDs.get("shooter-pivot"), MotorType.kBrushless);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        absolutePivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        absolutePivotEncoder.setPositionConversionFactor(360);

        pivotPID = pivotMotor.getPIDController();
        pivotPID.setFeedbackDevice(absolutePivotEncoder);
        pivotPID.setP(0.01);
        pivotPID.setI(0);
        pivotPID.setD(0);
        pivotPID.setFF(0.01);
    }

    /**
     * @param time The amount of time that the shooter motors run for, in seconds
     * @param speed The speed that the motors move (0 - 1)
     * @return The command that runs the shooter
     */
    // public Command runShooter(double time, double speed) {
    //     return run(() -> {
    //         topPID.setReference(speed / 1.25, CANSparkBase.ControlType.kVelocity);
    //         bottomPID.setReference(speed / 1.25, ControlType.kVelocity);
    //     }).withTimeout(time).andThen(runOnce(() -> {
    //         // Stop the motors when the time is up
    //         topPID.setReference(0, CANSparkBase.ControlType.kVelocity);
    //         bottomPID.setReference(0, ControlType.kVelocity);
    //     }));
    // }

    public Command toggleShooter(double speed) {
        return runOnce(() -> {
            if (topMotor.get() == 0) {
                topMotor.set(speed);
                bottomMotor.set(speed);
            } else {
                topMotor.set(0);
                bottomMotor.set(0);
            }
        });
    }

    public Command startShooter(double speed) {
        return runOnce(() -> {
            topMotor.set(speed);
            bottomMotor.set(speed);
        });
    }

    public Command stopShooter() {
        return runOnce(() -> {
            topMotor.set(0);
            bottomMotor.set(0);
        });
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
     * Stops the pivot by setting the target to its
     * current position.
     * @return The command that stops the pivot
     */
    public Command homePivot() {
        return runOnce(() -> {
            pivotTarget = 22;
        });
    }

    @Override
    public void periodic() {
        super.periodic();

        // Calculate the target and current rotations
        double encoderPos = absolutePivotEncoder.getPosition();
        Rotation2d target = Rotation2d.fromDegrees(pivotTarget);
        Rotation2d current = Rotation2d.fromDegrees(encoderPos);
        
        // Use the target and current to go to the 
        pivotMotor.set(-Math.max(Math.min(target.minus(current).getDegrees() / 120,
            /* Max motor speed */ 0.15), /* Min motor speed */ -0.15));

        // Display values
        SmartDashboard.putNumber("Shooter Pivot Encoder Position", encoderPos);
        SmartDashboard.putNumber("Shooter Pivot Encoder Target", pivotTarget);
        SmartDashboard.updateValues();
    }

    public Command togglePivot() {
        return runOnce(() -> {
            if (pivotTarget <= 22) {
                pivotTarget = 64;
            } else pivotTarget = 22;
        });
    }
}
