package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;
import frc.robot.util.PID;

public class TrapperSubsystem extends SubsystemBase {
    public static class Constants {
        public static double ARM_HOME_ANGLE = 0.05928;
        public static double PIVOT_HOME_ANGLE = 0.87801;

        public static double ARM_FEED_ANGLE = 0.75305;
        public static double PIVOT_FEED_ANGLE = 0.3492;

        public static double ARM_TRAP_ANGLE = 0.08189;
        public static double PIVOT_TRAP_ANGLE = 0.83944;

        public static PID TRAPPER_PID = new PID(0.1, 0, 0.01, 0);
    }

    private CANSparkMax rollers;

    private CANSparkMax pivotMotor;
    private AbsoluteEncoder pivotEncoder;

    private CANSparkMax armMotor;
    private AbsoluteEncoder armEncoder;

    // TODO: All of this needs redone with real values
    private Mechanism2d trapperMech = new Mechanism2d(3, 3);
    private MechanismRoot2d trapperRoot = trapperMech.getRoot("trapper", 1.5, 0);
    private MechanismLigament2d trapperArmMech;
    private MechanismLigament2d trapperWristMech;

    private double armTarget = Constants.ARM_HOME_ANGLE;
    private double pivotTarget = Constants.PIVOT_HOME_ANGLE;

    public TrapperSubsystem() {
        super();

        // Pivot (elbow)
        pivotMotor = new CANSparkMax(CanIDs.get("trapper-pivot"), MotorType.kBrushless);
        pivotMotor.setInverted(false);

        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        pivotEncoder.setZeroOffset(0);

        // Arm (shoulder)
        armMotor = new CANSparkMax(CanIDs.get("trapper-arm"), MotorType.kBrushless);
        armMotor.setInverted(true);

        armEncoder = armMotor.getAbsoluteEncoder();
        armEncoder.setZeroOffset(0);

        // Rollers (self-explanatory)
        rollers = new CANSparkMax(CanIDs.get("trapper-motor"), MotorType.kBrushless);

        // Mechanism2d stuff
        trapperArmMech = trapperRoot.append(new MechanismLigament2d("arm", 3, 0));
        trapperWristMech = trapperArmMech.append(new MechanismLigament2d("wrist", 1, 0));

        SmartDashboard.putData("Trapper Mechanism", trapperMech);
    }

    public Command runMotor() {
        return runOnce(() -> rollers.set(0.2));
    }

    public Command stopMotor() {
        return runOnce(() -> rollers.stopMotor());
    }

    public Command setPivotTarget(double target) {
        return runOnce(() -> {
            pivotTarget = target;
            //pivotPID.setReference(target, CANSparkBase.ControlType.kPosition);
        });
    }

    public Command setArmTarget(double target) {
        return runOnce(() -> {
            armTarget = target;
            // armPID.setReference(target, CANSparkBase.ControlType.kPosition);
        });
    }

    public Command setTargets(double armTarget, double pivotTarget) {
        return setPivotTarget(pivotTarget).alongWith(setArmTarget(armTarget));
    }

    public Command moveToFeedPosition() {
        return setPivotTarget(Constants.PIVOT_FEED_ANGLE).andThen(setArmTarget(Constants.ARM_FEED_ANGLE));
    }

    public Command moveToHomePosition() {
        return setPivotTarget(Constants.PIVOT_HOME_ANGLE).andThen(setArmTarget(Constants.ARM_HOME_ANGLE));
    }

    @Override
    public void periodic() {
        super.periodic();
        trapperArmMech.setAngle(armEncoder.getPosition() * 360);
        trapperWristMech.setAngle(pivotEncoder.getPosition() * 360);

        double pivotEncoderPos = pivotEncoder.getPosition();
        Rotation2d pivotTargetRotation = Rotation2d.fromRotations(pivotTarget);
        Rotation2d pivotCurrentRotation = Rotation2d.fromRotations(pivotEncoderPos);
        double pivotError = Math.max(Math.min(pivotTargetRotation.minus(pivotCurrentRotation).getDegrees() / 160.0,
            /* Max motor speed */ 0.3), /* Min motor speed */ -0.3);
        if (Math.abs(pivotError) < 0.01) pivotError = 0;
        pivotMotor.set(pivotError);

        double armEncoderPos = armEncoder.getPosition();
        Rotation2d armTargetRotation = Rotation2d.fromRotations(armTarget);
        Rotation2d armCurrentRotation = Rotation2d.fromRotations(armEncoderPos);
        double armError = Math.max(Math.min(armTargetRotation.minus(armCurrentRotation).getDegrees() / 200.0,
            /* Max motor speed */ 0.23), /* Min motor speed */ -0.23);
        if (Math.abs(armError) < 0.01) armError = 0;
        armMotor.set(armError);

        SmartDashboard.putNumber("Trapper Pivot Encoder Position", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Trapper Arm Encoder Position", armEncoder.getPosition());
        SmartDashboard.updateValues();
    }
}
