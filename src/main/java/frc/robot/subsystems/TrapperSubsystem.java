package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;
import frc.robot.util.PID;

public class TrapperSubsystem extends SubsystemBase {
    public static class Constants {
        public static double ARM_HOME_ANGLE = 149.05;
        public static double WRIST_HOME_ANGLE = 10.32;

        public static double ARM_FEED_ANGLE = 50.40;
        public static double WRIST_FEED_ANGLE = 185.35;

        public static double ARM_TRAP_ANGLE = 0.08189;
        public static double WRIST_TRAP_ANGLE = 0.83944;

        public static PID TRAPPER_PID = new PID(0.1, 0, 0.01, 0);
    }

    private CANSparkMax rollers;

    private CANSparkMax wristMotor;
    private AbsoluteEncoder wristEncoder;

    private CANSparkMax armMotor;
    private AbsoluteEncoder armEncoder;

    // TODO: All of this needs redone with real values
    private Mechanism2d trapperMech = new Mechanism2d(38, 40);
    private MechanismRoot2d trapperRoot = trapperMech.getRoot("trapper", 12, 20);
    private MechanismLigament2d trapperArmMech;
    private MechanismLigament2d trapperWristMech;

    private double armTarget = Constants.ARM_HOME_ANGLE;
    private double wristTarget = Constants.WRIST_HOME_ANGLE;

    public TrapperSubsystem() {
        super();

        // Wrist (the end part)
        wristMotor = new CANSparkMax(CanIDs.get("trapper-wrist"), MotorType.kBrushless);
        wristMotor.setInverted(true);

        wristEncoder = wristMotor.getAbsoluteEncoder();
        wristEncoder.setPositionConversionFactor(360);
        wristEncoder.setZeroOffset(300);

        // Arm (the bottom part)
        armMotor = new CANSparkMax(CanIDs.get("trapper-arm"), MotorType.kBrushless);
        armMotor.setInverted(true);

        armEncoder = armMotor.getAbsoluteEncoder();
        armEncoder.setPositionConversionFactor(360);
        armEncoder.setZeroOffset(0);

        // Rollers (self-explanatory)
        rollers = new CANSparkMax(CanIDs.get("trapper-motor"), MotorType.kBrushless);

        // Mechanism2d stuff
        trapperArmMech = trapperRoot.append(new MechanismLigament2d("arm", 10, 0, 1, new Color8Bit(Color.kCoral)));
        trapperWristMech = trapperArmMech.append(new MechanismLigament2d("wrist", 16, 0));

        SmartDashboard.putData("Trapper Mechanism", trapperMech);
    }

    public Command runMotor() {
        return runOnce(() -> rollers.set(0.2));
    }

    public Command stopMotor() {
        return runOnce(() -> rollers.stopMotor());
    }

    public Command setWristTarget(double target) {
        return runOnce(() -> {
            wristTarget = target;
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
        return setWristTarget(pivotTarget).alongWith(setArmTarget(armTarget));
    }

    public Command moveToFeedPosition() {
        return setWristTarget(Constants.WRIST_FEED_ANGLE).andThen(setArmTarget(Constants.ARM_FEED_ANGLE));
    }

    public Command moveToHomePosition() {
        return setWristTarget(Constants.WRIST_HOME_ANGLE).andThen(setArmTarget(Constants.ARM_HOME_ANGLE));
    }

    @Override
    public void periodic() {
        super.periodic();
        trapperArmMech.setAngle(armEncoder.getPosition());
        trapperWristMech.setAngle(wristEncoder.getPosition());

        // Target the proper angles

        double wristEncoderPos = wristEncoder.getPosition();
        Rotation2d wristTargetRotation = Rotation2d.fromDegrees(wristTarget);
        Rotation2d wristCurrentRotation = Rotation2d.fromDegrees(wristEncoderPos);
        double wristError = Math.max(Math.min((wristTargetRotation.minus(wristCurrentRotation).getDegrees()) / 400.0,
            /* Max motor speed */ 0.15), /* Min motor speed */ -0.15);
        wristMotor.set(wristError);

        double armEncoderPos = armEncoder.getPosition();
        Rotation2d armTargetRotation = Rotation2d.fromDegrees(armTarget);
        Rotation2d armCurrentRotation = Rotation2d.fromDegrees(armEncoderPos);
        double armError = Math.max(Math.min(armTargetRotation.minus(armCurrentRotation).getDegrees() / 300.0,
            /* Max motor speed */ 0.25), /* Min motor speed */ -0.25);
        // if (Math.abs(armError) < 0.01) armError = 0;
        armMotor.set(armError);

        SmartDashboard.putNumber("Trapper Wrist Encoder Target", wristTarget);
        SmartDashboard.putNumber("Trapper Arm Encoder Target", armTarget);
        SmartDashboard.putNumber("Trapper Wrist Encoder Position", wristEncoder.getPosition());
        SmartDashboard.putNumber("Trapper Arm Encoder Position", armEncoder.getPosition());
        SmartDashboard.updateValues();
    }
}