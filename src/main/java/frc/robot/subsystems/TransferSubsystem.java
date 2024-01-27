package frc.robot.subsystems;

import frc.robot.util.CanIDs;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

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

    /**
     * Command to intake a NOTE into the transfer, which runs until a stop condition is met or a note is in the transfer
     * @param earlyStop Condition to stop early
     * @return The command
     */
    public Command intakeCommand(BooleanSupplier earlyStop) {
        return runEnd(
            () -> beltMotor.set(.5),
            () -> {
                beltMotor.set(0);
                occupied = beamBreak.get();
            }
        ).until(() -> beamBreak.get() || earlyStop.getAsBoolean()); // Stop when the beam breaks
    }

    /**
     * Command to shut off the motor
     * @return The command
     */
    public Command shutoffCommand() {
        return runOnce(() -> beltMotor.stopMotor());
    }

    /**
     * Command to feed the NOTE into the shooter
     * @return The command
     */
    public Command feedCommand() {
        return runEnd(
            () -> beltMotor.set(.5),
            () -> {
                beltMotor.set(0);
                occupied = false;
            }
        ).withTimeout(.5); // Stop after a timeout
    }

    /**
     * Get whether the transfer has a NOTE in it
     * @return whether the transfer is occupied
     */
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
