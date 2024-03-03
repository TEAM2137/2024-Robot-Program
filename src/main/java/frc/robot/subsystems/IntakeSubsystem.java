package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;

import frc.robot.util.CanIDs;
import frc.robot.util.PID;

// everything number is a placeholder
public class IntakeSubsystem extends SubsystemBase {

    public static class Constants {
        public static PID rollerPID = new PID(0.1, 0.2, 0.3, 0.4);
        public static PID pivotPID = new PID(0.01, 0.02, 0.03, 0.04);
    }

    // private double currentThreshold = 75;

    private double minPos = 160.0;
    private double maxPos = 306.8;

    private double pivotTarget = maxPos;
    
    private CANSparkMax pivotMotor;
    private CANSparkMax rollerMotor;

    private AbsoluteEncoder pivotEncoder;

    private boolean isRaised;
  
    public IntakeSubsystem() {
        super();

        rollerMotor = new CANSparkMax(CanIDs.get("intake-rollers"), CANSparkLowLevel.MotorType.kBrushless);

        pivotMotor = new CANSparkMax(CanIDs.get("intake-pivot"), CANSparkLowLevel.MotorType.kBrushless);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setInverted(false);

        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        pivotEncoder.setPositionConversionFactor(360);
        pivotEncoder.setZeroOffset(180);
    }

    public void init() {
        rollerMotor.stopMotor();
        pivotMotor.stopMotor();
        pivotTarget = maxPos;
    }

    public Command startRollers() {
        return runOnce(() -> rollerMotor.set(-1));
    }

    public Command stopRollers() {
        return runOnce(() -> rollerMotor.set(0));
    }

    public Command moveIntakeDown() {
        return runOnce(() -> {
            pivotTarget = minPos;
            isRaised = false;
        });
    }

    public Command moveIntakeUp() {
        return runOnce(() -> {
            pivotTarget = maxPos;
            isRaised = true;
        });
    }

    @Override
    public void periodic() {
        super.periodic();

        double encoderPos = pivotEncoder.getPosition();
        Rotation2d target = Rotation2d.fromDegrees(pivotTarget);
        Rotation2d current = Rotation2d.fromDegrees(encoderPos);

        double error = Math.max(Math.min(target.minus(current).getDegrees() / 320.0,
            /* Max motor speed */ 0.5), /* Min motor speed */ -0.5);

        if (error < 0) error /= 2; // Reduce power going down
        
        pivotMotor.set(error);

        SmartDashboard.putNumber("Intake Encoder Position", pivotEncoder.getPosition());
        SmartDashboard.updateValues();
    }

    public Command togglePivot() {
        return runOnce(() -> {
            if (isRaised) CommandScheduler.getInstance().schedule(moveIntakeDown());
            else CommandScheduler.getInstance().schedule(moveIntakeUp());
        });
    }

    public Command deployIntake() {
        return startRollers().andThen(moveIntakeDown());
    }
}
