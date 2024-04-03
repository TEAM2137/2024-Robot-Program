package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;

public class TrapperSubsystem extends SubsystemBase {
    public static class Constants {
        public static final double ARM_HOME_ANGLE = 232.0;
        public static final double WRIST_HOME_ANGLE = 261.3;

        public static final double ARM_AMP_ANGLE = 145.6;
        public static final double WRIST_AMP_ANGLE = 150.2;
    }

    private CANSparkMax rollers;

    private CANSparkMax wristMotor;
    private AbsoluteEncoder wristEncoder;

    private CANSparkMax armMotor;
    private AbsoluteEncoder armEncoder;

    private DigitalInput noteSensor;

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
        wristMotor.setIdleMode(IdleMode.kBrake);

        wristEncoder = wristMotor.getAbsoluteEncoder();
        wristEncoder.setPositionConversionFactor(360);
        wristEncoder.setZeroOffset(100);

        // Arm (the bottom part)
        armMotor = new CANSparkMax(CanIDs.get("trapper-arm"), MotorType.kBrushless);
        armMotor.setInverted(true);
        armMotor.setIdleMode(IdleMode.kBrake);

        armEncoder = armMotor.getAbsoluteEncoder();
        armEncoder.setPositionConversionFactor(360);
        armEncoder.setZeroOffset(300);

        // Rollers (self-explanatory)
        rollers = new CANSparkMax(CanIDs.get("trapper-motor"), MotorType.kBrushless);

        // Distance sensor looking for note
        noteSensor = new DigitalInput(3);

        // Mechanism2d stuff
        trapperArmMech = trapperRoot.append(new MechanismLigament2d("arm", 10, 0, 1, new Color8Bit(Color.kCoral)));
        trapperWristMech = trapperArmMech.append(new MechanismLigament2d("wrist", 16, 0));

        SmartDashboard.putData("Trapper Mechanism", trapperMech);
    }

    public Command runRollers(double speed) {
        return runOnce(() -> rollers.set(speed));
    }

    public Command stopRollers() {
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
        return setWristTarget(Constants.WRIST_HOME_ANGLE).andThen(Commands.waitSeconds(0.25))
            .andThen(setArmTarget(Constants.ARM_HOME_ANGLE));
    }

    public Command ampPosition() {
        return setArmTarget(Constants.ARM_AMP_ANGLE).andThen(Commands.waitSeconds(0.25))
            .andThen(setWristTarget(Constants.WRIST_AMP_ANGLE));
    }

    public Command climbPosition() {
        return setArmTarget(Constants.ARM_AMP_ANGLE).andThen(setWristTarget(Constants.WRIST_HOME_ANGLE));
    }

    @Override
    public void periodic() {
        super.periodic();
        trapperArmMech.setAngle(armEncoder.getPosition());
        trapperWristMech.setAngle(wristEncoder.getPosition());

        // Target the proper angles

        double wristEncoderPos = wristEncoder.getPosition();
        double wristError = Math.max(Math.min((wristTarget - wristEncoderPos) / 390.0,
            /* Max motor speed */ 0.16), /* Min motor speed */ -0.16);
        wristMotor.set(wristError);

        double armEncoderPos = armEncoder.getPosition();
        double armError = Math.max(Math.min((armTarget - armEncoderPos) / 300.0,
            /* Max motor speed */ 0.28), /* Min motor speed */ -0.28);
        armMotor.set(armError);

        SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
        SmartDashboard.putNumber("Arm Position", armEncoder.getPosition());
        SmartDashboard.updateValues();
    }

    public BooleanSupplier beamBroken() {
        return () -> noteSensor.get();
    }

    public BooleanSupplier beamClear() {
        return () -> !noteSensor.get();
    }
}