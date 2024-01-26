package frc.robot.subsystems;

import frc.robot.util.CanIDs;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TransferSubsystem extends SubsystemBase {
    private boolean occupied = false;

    private CANSparkMax beltMotor;
    private DigitalInput beamBreak = new DigitalInput(0);

    public TransferSubsystem() {
        super();

        beltMotor = new CANSparkMax(CanIDs.get("transfer-motor"), CANSparkLowLevel.MotorType.kBrushless);
    }

    public Command intakeCommand(BooleanSupplier earlyStop) {
        return runEnd(
            () -> beltMotor.set(.5),
            () -> {
                beltMotor.set(0);
                occupied = beamBreak.get();
            }
        ).until(() -> beamBreak.get() || earlyStop.getAsBoolean()); // Stop when the beam breaks
    }

    public Command shutoffCommand() {
        return runOnce(() -> beltMotor.stopMotor());
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
        SmartDashboard.putBoolean("Beam Break Output", beamBreak.get());
        SmartDashboard.updateValues();
    }
}
