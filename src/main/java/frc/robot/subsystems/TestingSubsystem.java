package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.CanIDs;

public class TestingSubsystem extends SubsystemBase {
    private CANSparkMax testMotor = new CANSparkMax(CanIDs.get("testing-motor"), MotorType.kBrushless);

    public Command testMotorOn() {
        return runOnce(() -> testMotor.set(.1));
    }

    public Command testMotorOff() {
        return runOnce(() -> testMotor.set(0));
    }
}
