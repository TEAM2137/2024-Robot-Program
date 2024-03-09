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
        public static double pivotKP = 0.00356; // Power the pivot moves to its target with
        public static double rollersKP = 0.0001; // Power the rollers move to their targets with
        public static double pivotMotorLimit = 0.4; // Max power of the pivot motor
        public static double rollerMotorLimit = 0.75; // Max power of the pivot motor
        public static double gravityMod = 0.85; // Reduces power moving the pivot down

        public static PID rollerPID = new PID(0.1, 0.2, 0.3, 0.4);
        public static PID pivotPID = new PID(0.01, 0.02, 0.03, 0.04);
    }

    // private double currentThreshold = 75;

    private double minPos = 145.0;
    private double maxPos = 315.0;

    private double pivotTarget = maxPos;
    private double rollerTarget = 0;
    
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
        pivotEncoder.setZeroOffset(280);
    }

    public void init() {
        rollerMotor.stopMotor();
        pivotMotor.stopMotor();
        rollerTarget = 0;
        pivotTarget = maxPos;
    }

    public Command startRollers() {
        return runOnce(() -> rollerTarget = -1);
    }

    public Command reverseRollers() {
        return runOnce(() -> rollerTarget = 1);
    }

    public Command stopRollers() {
        return runOnce(() -> rollerTarget = 0);
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

        Rotation2d pivotTargetRotation = Rotation2d.fromDegrees(pivotTarget);
        Rotation2d pivotRotation = Rotation2d.fromDegrees(pivotEncoder.getPosition());

        double pivotError = Math.max(Math.min(pivotTargetRotation.minus(pivotRotation).getDegrees() * Constants.pivotKP,
            /* Max motor speed */ Constants.pivotMotorLimit), /* Min motor speed */ -Constants.pivotMotorLimit);
        if (pivotError < 0) pivotError *= Constants.gravityMod; // Reduce power going down
        pivotMotor.set(pivotError);

        double rollersError = Math.max(Math.min(rollerTarget - rollerMotor.get() * Constants.pivotKP,
            /* Max motor speed */ Constants.pivotMotorLimit), /* Min motor speed */ -Constants.pivotMotorLimit);
        rollerMotor.set(rollersError);

        SmartDashboard.putNumber("Intake Position", pivotEncoder.getPosition());
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
