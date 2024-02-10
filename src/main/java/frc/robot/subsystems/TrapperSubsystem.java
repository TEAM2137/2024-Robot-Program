package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;

public class TrapperSubsystem extends SubsystemBase {
    private CANSparkMax trapperMotor;

    private CANSparkMax pivotMotor;
    private RelativeEncoder pivotEncoder;
    private DutyCycleEncoder pivotAbsoluteEncoder;
    private SparkPIDController pivotPID;

    private CANSparkMax armMotor;
    private RelativeEncoder armEncoder;
    private DutyCycleEncoder armAbsoluteEncoder;
    private SparkPIDController armPID;

    public TrapperSubsystem() {
        super();
        trapperMotor = new CANSparkMax(CanIDs.get("trapper-motor"), MotorType.kBrushless);

        // TODO: Get actual channels for absolute encoders
        pivotAbsoluteEncoder = new DutyCycleEncoder(4);
        armAbsoluteEncoder = new DutyCycleEncoder(5);

        // TODO: Set absolute encoder offsets
        pivotAbsoluteEncoder.setPositionOffset(0);
        armAbsoluteEncoder.setPositionOffset(0);

        pivotMotor = new CANSparkMax(CanIDs.get("trapper-pivot"), MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPID = pivotMotor.getPIDController();
        pivotPID.setFeedbackDevice(pivotEncoder);

        armMotor = new CANSparkMax(CanIDs.get("trapper-arm"), MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
        armPID = armMotor.getPIDController();
        armPID.setFeedbackDevice(armEncoder);
    }

    public Command runMotor() {
        return runOnce(() -> trapperMotor.set(.5));
    }

    public Command stopMotor() {
        return runOnce(() -> trapperMotor.stopMotor());
    }

    public Command setPivotTarget(double target) {
        return runOnce(() -> {
            pivotEncoder.setPosition(pivotAbsoluteEncoder.getDistance()); // Definitely a hack. No idea what else to do
            pivotPID.setReference(target / 360, CANSparkBase.ControlType.kPosition);
        });
    }

    public Command setArmTarget(double target) {
        return runOnce(() -> {
            armEncoder.setPosition(armAbsoluteEncoder.getDistance());
            armPID.setReference(target / 360, CANSparkBase.ControlType.kPosition);
        });
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Trapper Pivot Encoder Position", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Trapper Arm Encoder Position", armEncoder.getPosition());
        SmartDashboard.updateValues();
    }
}
