package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;

public class TrapperSubsystem extends SubsystemBase {
    public static class Constants {
        public static double ARM_FEED_ANGLE = 0; //TODO: add real angle based on measurement
        public static double PIVOT_FEED_ANGLE = 0; //TODO: add real angle based on measurement
    }

    private CANSparkMax rollers;

    private CANSparkMax pivotMotor;
    private AbsoluteEncoder pivotEncoder;
    private SparkPIDController pivotPID;

    private CANSparkMax armMotor;
    private AbsoluteEncoder armEncoder;
    private SparkPIDController armPID;

    // TODO: All of this needs redone with real values
    private Mechanism2d trapperMech = new Mechanism2d(3, 3);
    private MechanismRoot2d trapperRoot = trapperMech.getRoot("trapper", 1.5, 0);
    private MechanismLigament2d trapperArmMech;
    private MechanismLigament2d trapperWristMech;

    public TrapperSubsystem() {
        super();
        rollers = new CANSparkMax(CanIDs.get("trapper-motor"), MotorType.kBrushless);

        pivotMotor = new CANSparkMax(CanIDs.get("trapper-pivot"), MotorType.kBrushless);
        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        pivotEncoder.setZeroOffset(0); // TODO: set actual offset

        pivotPID = pivotMotor.getPIDController();
        pivotPID.setFeedbackDevice(pivotEncoder);

        armMotor = new CANSparkMax(CanIDs.get("trapper-arm"), MotorType.kBrushless);
        armEncoder = armMotor.getAbsoluteEncoder();
        armEncoder.setZeroOffset(0); // TODO: set actual offset

        armPID = armMotor.getPIDController();
        armPID.setFeedbackDevice(armEncoder);

        trapperArmMech = trapperRoot.append(new MechanismLigament2d("arm", 3, 0));
        trapperWristMech = trapperArmMech.append(new MechanismLigament2d("wrist", 1, 0));

        SmartDashboard.putData("Trapper Mechanism", trapperMech);
    }

    public Command runMotor() {
        return runOnce(() -> rollers.set(.2));
    }

    public Command stopMotor() {
        return runOnce(() -> rollers.stopMotor());
    }

    public Command setPivotTarget(double target) {
        return runOnce(() -> {
            pivotPID.setReference(target / 360, CANSparkBase.ControlType.kPosition);
        });
    }

    public Command setArmTarget(double target) {
        return runOnce(() -> {
            armPID.setReference(target / 360, CANSparkBase.ControlType.kPosition);
        });
    }

    public Command moveToFeedPosition() {
        return setPivotTarget(Constants.PIVOT_FEED_ANGLE).alongWith(setArmTarget(Constants.ARM_FEED_ANGLE));
    }

    @Override
    public void periodic() {
        super.periodic();
        trapperArmMech.setAngle(armEncoder.getPosition() * 360);
        trapperWristMech.setAngle(pivotEncoder.getPosition() * 360);

        SmartDashboard.putNumber("Trapper Pivot Encoder Position", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Trapper Arm Encoder Position", armEncoder.getPosition());
        SmartDashboard.updateValues();
    }
}
