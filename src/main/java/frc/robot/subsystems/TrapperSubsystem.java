package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
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
        public static double ARM_HOME_ANGLE = 51.9;
        public static double WRIST_HOME_ANGLE = 109.8;

        public static double ARM_AMP = 7.3;
        public static double WRIST_AMP = 182.4;

        public static double ARM_GRAB = 7.3;
        public static double WRIST_GRAB = 182.4;

        public static PID TRAPPER_PID = new PID(0.1, 0, 0.01, 0);
    }

    private CANSparkMax rollers;

    private CANSparkMax wristMotor;
    private AbsoluteEncoder wristEncoder;

    private CANSparkMax armMotor;
    private AbsoluteEncoder armEncoder;

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
        wristMotor.setInverted(false);
        wristMotor.setIdleMode(IdleMode.kBrake);

        wristEncoder = wristMotor.getAbsoluteEncoder();
        wristEncoder.setPositionConversionFactor(360);
        wristEncoder.setZeroOffset(200);

        // Arm (the bottom part)
        armMotor = new CANSparkMax(CanIDs.get("trapper-arm"), MotorType.kBrushless);
        armMotor.setInverted(true);
        armMotor.setIdleMode(IdleMode.kBrake);

        armEncoder = armMotor.getAbsoluteEncoder();
        armEncoder.setPositionConversionFactor(360);
        armEncoder.setZeroOffset(300);

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
        });
    }

    public Command setArmTarget(double target) {
        return runOnce(() -> {
            armTarget = target;
        });
    }

    public Command setTargets(double armTarget, double pivotTarget) {
        return setWristTarget(pivotTarget).alongWith(setArmTarget(armTarget));
    }

    public Command homePosition() {
        return setWristTarget(Constants.WRIST_HOME_ANGLE).andThen(setArmTarget(Constants.ARM_HOME_ANGLE));
    }

    public Command ampPosition() {
        return setWristTarget(Constants.WRIST_AMP).andThen(setArmTarget(Constants.ARM_AMP));
    }

    public Command grabPosition() {
        return setWristTarget(Constants.WRIST_GRAB).andThen(setArmTarget(Constants.ARM_GRAB));
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

        // SmartDashboard.putNumber("Trapper Wrist Encoder Target", wristTarget);
        // SmartDashboard.putNumber("Trapper Arm Encoder Target", armTarget);
        SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
        SmartDashboard.putNumber("Arm Position", armEncoder.getPosition());
        SmartDashboard.updateValues();
    }
}