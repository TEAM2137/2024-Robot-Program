package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;
 
public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;

    private SparkPIDController topPID;
    private SparkPIDController bottomPID;

    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder;

    private CANSparkMax pivotMotor;

    private DutyCycleEncoder absolutePivotEncoder;

    private RelativeEncoder relativePivotEncoder;
    private SparkPIDController pivotPID;

    public ShooterSubsystem() {
        super();
        topMotor = new CANSparkMax(CanIDs.get("shooter-top"), MotorType.kBrushless);
        bottomMotor = new CANSparkMax(CanIDs.get("shooter-bottom"), MotorType.kBrushless);

        topEncoder = topMotor.getEncoder();
        bottomEncoder = bottomMotor.getEncoder();
        topPID = topMotor.getPIDController();
        bottomPID = bottomMotor.getPIDController();
        topPID.setFeedbackDevice(topEncoder);
        bottomPID.setFeedbackDevice(bottomEncoder);

        absolutePivotEncoder = new DutyCycleEncoder(3); // TODO replace with actual DIO input
        absolutePivotEncoder.setPositionOffset(0); // TODO set encoder offset
        
        relativePivotEncoder = pivotMotor.getEncoder();
        pivotMotor = new CANSparkMax(CanIDs.get("shooter-pivot"), MotorType.kBrushless);
        pivotPID = pivotMotor.getPIDController();
        pivotPID.setFeedbackDevice(relativePivotEncoder);
    }

    /**
     * @param time The amount of time that the shooter motors run for, in seconds
     * @param speed The speed that the motors move (0 - 1)
     * @return The command that runs the shooter
     */
    public Command runShooter(double time, double speed) {
        return run(() -> {
            topPID.setReference(speed / 1.25, CANSparkBase.ControlType.kVelocity);
            bottomPID.setReference(speed / 1.25, ControlType.kVelocity);
        }).withTimeout(time).andThen(runOnce(() -> {
            // Stop the motors when the time is up
            topPID.setReference(0, CANSparkBase.ControlType.kVelocity);
            bottomPID.setReference(0, ControlType.kVelocity);
        }));
    }

    public Command startShooter(double speed) {
        return runOnce(() -> {
            topPID.setReference(speed, CANSparkBase.ControlType.kVelocity);
            bottomPID.setReference(speed, ControlType.kVelocity);
        });
    }

    public Command stopShooter() {
        return runOnce(() -> {
            topPID.setReference(0, CANSparkBase.ControlType.kVelocity);
            bottomPID.setReference(0, ControlType.kVelocity);
        });
    }

    /**
     * Set the position of the shooter pivot
     * @param target The target angle in degrees
     * @return The command that moves the pivot
     */
    public Command setPivotTarget(double target) {
        return runOnce(() -> {
            relativePivotEncoder.setPosition(absolutePivotEncoder.getAbsolutePosition());
            pivotPID.setReference(target / 360, CANSparkBase.ControlType.kPosition);
        });
    }

    @Override
    public void periodic() {
        super.periodic();
        // Apply the absolute encoder's position to the relative encoder
        relativePivotEncoder.setPosition(absolutePivotEncoder.getAbsolutePosition());

        // Display values
        SmartDashboard.putNumber("Shooter Pivot Encoder Position", absolutePivotEncoder.getAbsolutePosition());
        SmartDashboard.updateValues();
    }
}
