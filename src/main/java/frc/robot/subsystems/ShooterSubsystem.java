package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;
 
public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;

    private SparkPIDController shooterMotor1PID;
    private SparkPIDController shooterMotor2PID;
    private RelativeEncoder shooterEncoder1;
    private RelativeEncoder shooterEncoder2;

    private CANSparkMax pivotMotor;
    private SparkAbsoluteEncoder pivotEncoder;
    private SparkPIDController pivotPID;

    public ShooterSubsystem() {
        super();
        shooterMotor1 = new CANSparkMax(CanIDs.get("shooter-1"), MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(CanIDs.get("shooter-2"), MotorType.kBrushless);
        shooterEncoder1 = shooterMotor1.getEncoder();
        shooterEncoder2 = shooterMotor2.getEncoder();
        shooterMotor1PID = shooterMotor1.getPIDController();
        shooterMotor2PID = shooterMotor2.getPIDController();

        pivotMotor = new CANSparkMax(CanIDs.get("shooter-pivot"), MotorType.kBrushless);
        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pivotPID = pivotMotor.getPIDController();
        pivotPID.setFeedbackDevice(pivotEncoder);
        shooterMotor1PID.setFeedbackDevice(shooterEncoder1);
        shooterMotor2PID.setFeedbackDevice(shooterEncoder2);
    }

    /**
     * @param time The amount of time that the shooter motors run for, in seconds
     * @param speed The speed that the motors move (0 - 1)
     * @return The command that runs the shooter
     */
    public Command runShooter(double time, double speed) {
        return run(() -> {
            shooterMotor1PID.setReference(speed / 1.25, CANSparkBase.ControlType.kVelocity);
            shooterMotor2PID.setReference(speed / 1.25, ControlType.kVelocity);
        }).withTimeout(time).andThen(runOnce(() -> {
            // Stop the motors when the time is up
            shooterMotor1PID.setReference(0, CANSparkBase.ControlType.kVelocity);
            shooterMotor2PID.setReference(0, ControlType.kVelocity);
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
