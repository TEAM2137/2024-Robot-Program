package frc.robot.subsystems;

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
import frc.robot.util.PID;
 
public class ShooterSubsystem extends SubsystemBase {

    public static class Constants {
        public static PID pivotPID = new PID(0.0001, 0.0, 0.001, 0);

        public static int lowAngle = 15;
        public static int highAngle = 34;
    }

    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;
    private CANSparkMax pivotMotor;

    //private SparkPIDController pivotPID;

    private AbsoluteEncoder absolutePivotEncoder;
    private double pivotTarget;
    private boolean running;

    public ShooterSubsystem() {
        super();

        pivotTarget = 34;

        topMotor = new CANSparkMax(CanIDs.get("shooter-top"), MotorType.kBrushless);
        bottomMotor = new CANSparkMax(CanIDs.get("shooter-bottom"), MotorType.kBrushless);
        topMotor.stopMotor();
        bottomMotor.stopMotor();

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

    public Command toggleShooter(double speed) {
        return runOnce(() -> {
            if (!running) CommandScheduler.getInstance().schedule(startShooter(speed));
            else CommandScheduler.getInstance().schedule(stopShooter());
        });
    }

    public Command startShooter(double speed) {
        return runOnce(() -> {
            topMotor.set(speed);
            bottomMotor.set(speed);
        }).andThen(() -> running = true);
    }

    public Command stopShooter() {
        return runOnce(() -> {
            topMotor.set(0);
            bottomMotor.set(0);
        }).andThen(() -> running = false);
    }

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
        SmartDashboard.putNumber("Shooter Pivot Encoder Position", absolutePivotEncoder.getPosition());
        SmartDashboard.putNumber("Shooter Pivot Encoder Target", pivotTarget);
        SmartDashboard.updateValues();
    }
}