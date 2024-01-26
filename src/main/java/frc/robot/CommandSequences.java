package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class CommandSequences {

    public CommandSequences() {
        // Leaving this here in case we need to store variables between commands
    }

    public static Command startIntakeCommand(IntakeSubsystem intake, TransferSubsystem transfer, Trigger button) {
        return intake.StartMotors().alongWith(transfer.intakeCommand(button));
    }
}