package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;
 
public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;

    private CANSparkMax pivotMotor;
    private SparkAbsoluteEncoder pivotEncoder;
    private SparkPIDController pivotPID;

    public ShooterSubsystem() {
        super();
        shooterMotor1 = new CANSparkMax(CanIDs.get("shooter-1"), MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(CanIDs.get("shooter-2"), MotorType.kBrushless);

        pivotMotor = new CANSparkMax(CanIDs.get("shooter-pivot"), MotorType.kBrushless);
        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pivotPID = pivotMotor.getPIDController();
        pivotPID.setFeedbackDevice(pivotEncoder);
    }

    /**
     * @param time The amount of time that the shooter motors run for, in seconds
     * @param speed The speed that the motors move (0 - 1)
     * @return The command that runs the shooter
     */
    public Command runShooter(double time, double speed) {
        return run(() -> {
            shooterMotor1.set(speed);
            shooterMotor2.set(speed);
        }).withTimeout(time).andThen(runOnce(() -> {
            // Stop the motors when the time is up
            shooterMotor1.set(0);
            shooterMotor2.set(0);
        }));
    }

    /**
     * Set the position of the shooter pivot
     * @param target The target angle in degrees
     * @return The command that moves the pivot
     */
    public Command setPivotTarget(double target) {
        return runOnce(() -> pivotPID.setReference(target / 360, CANSparkBase.ControlType.kPosition));
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Shooter Pivot Encoder Position", pivotEncoder.getPosition());
        SmartDashboard.updateValues();
    }
}
