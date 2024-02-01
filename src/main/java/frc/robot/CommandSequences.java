package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TrapperSubsystem;

public class CommandSequences {

    public static Command startIntakeCommand(IntakeSubsystem intake, TransferSubsystem transfer, BooleanSupplier earlyStop) {
        return 
            intake.MoveIntakeDown() // Lower intake
            .andThen(intake.StartMotors()) // Start intake
            .alongWith(transfer.intakeCommand(earlyStop) // Start transfer
            .andThen(intake.PleaseStop())) // When transfer finishes stop the intake
            .andThen(intake.MoveIntakeUp()); // Raise intake again
    }

    public static Command raiseClimberCommand(ClimberSubsystem climb, IntakeSubsystem intake) {
        return
            intake.MoveIntakeDown()
            .andThen(climb.climberUpCommand());
    }

    public static Command lowerClimberCommand(ClimberSubsystem climb, IntakeSubsystem intake) {
        return
            intake.MoveIntakeDown()
            .andThen(climb.climberDownCommand());
    }

    public static Command transferToShooter(TransferSubsystem transfer, TrapperSubsystem trapper) {
        return
            transfer.feedShooterCommand()
            .alongWith(trapper.runMotor())
            .andThen(trapper.stopMotor());
    }

    public static Command transferToTrapper(TransferSubsystem transfer, TrapperSubsystem trapper) {
        return
            transfer.feedTrapperCommand()
            .alongWith(trapper.runMotor()) // I'm thinking to transfer it completely we'll need the trapper to intake a little bit
            .andThen(trapper.stopMotor());
    }
}
