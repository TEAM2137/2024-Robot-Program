package frc.robot.subsystems;

import frc.robot.util.CanIDs;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase {
    private boolean occupied = false;

    private CANSparkMax beltMotor;
    private DigitalInput beamBreak = new DigitalInput(0);

    public TransferSubsystem() {
        super();

        beltMotor = new CANSparkMax(CanIDs.get("transfer-motor"), CANSparkLowLevel.MotorType.kBrushless);
    }

    public Command intakeCommand() {
        return runEnd(
            () -> beltMotor.set(.5),
            () -> {
                beltMotor.set(0);
                occupied = true;
            }
        ).until(() -> beamBreak.get()); // Stop when the beam breaks
    }

    public Command feedCommand() {
        return runEnd(
            () -> beltMotor.set(.5),
            () -> {
                beltMotor.set(0);
                occupied = false;
            }
        ).withTimeout(.5); // Stop after a timeout
    }

    public boolean getOccupied() {
        return occupied;
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putBoolean("Transfer Occupied", occupied);
        SmartDashboard.updateValues();
    }
}
