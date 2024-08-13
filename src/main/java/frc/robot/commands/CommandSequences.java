package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Static class containing all of the necessary command sequences for auton/teleop
 */
public class CommandSequences {

    public static boolean isBlueAlliance;
    public static double calculatedShooterSpeed = 0.9;

    // +++ Shooter +++

    /**
     * Spins up the shooter motors to a specified speed and shoots with
     * whatever angle the shooter is currently at.
     * @return
     */
    public static Command rawShootCommand(double speed, TransferSubsystem transfer, ShooterSubsystem shooter) {
        return shooter.startAndRun(speed, 1.2)
            .andThen(startShooterAndTransfer(speed, shooter, transfer).withTimeout(0.8))
            .andThen(stopShooterAndTransfer(shooter, transfer));
    }

    /**
     * Shoots into the amp by aiming the shooter pivot and shooting
     * @return the command
     */
    public static Command ampShootCommand(double topSpeed, double bottomSpeed, TransferSubsystem transfer, ShooterSubsystem shooter) {
        return shooter.startSeperateShooters(topSpeed, bottomSpeed)
            .andThen(Commands.waitSeconds(1.2))
            .andThen(transfer.removeForceStop().andThen(transfer.feedShooterCommand()).withTimeout(0.8))
            .andThen(stopShooterAndTransfer(shooter, transfer));
    }
    // public static Command ampShootCommand(double speed, TransferSubsystem transfer, ShooterSubsystem shooter) {
    //     return shooter.startShooter(speed)
    //         .andThen(Commands.waitSeconds(1.2))
    //         .andThen(startShooterAndTransfer(speed, shooter, transfer).withTimeout(0.8))
    //         .andThen(stopShooterAndTransfer(shooter, transfer));
    // }

    public static Command transferToShooterCommand(IntakeSubsystem intake, TransferSubsystem transfer, ShooterSubsystem shooter, ArmSubsystem trapper) {
        return transfer.feedShooterCommand().withTimeout(0.8)
            .andThen(CommandSequences.stopAllSubsystems(intake, transfer, shooter, trapper));
    }
    
    /**
     * Starts both the shooter and transfer. You probably shouldn't call this
     * until the shooter motors are spun up first.
     * @return the command
     */
    public static Command startShooterAndTransfer(double speed, ShooterSubsystem shooter, TransferSubsystem transfer) {
        return shooter.startShooter(speed)
            .alongWith(transfer.removeForceStop().andThen(transfer.feedShooterCommand()));
    }

    /**
     * Force stops both the transfer and the shooter motors 
     * @return the command
     */
    public static Command stopShooterAndTransfer(ShooterSubsystem shooter, TransferSubsystem transfer) {
        return shooter.stopShooter().alongWith(transfer.transferForceStop());
    }

    /**
     * Starts the shooter motors for a specified amount of time. The 
     * motors will not stop when the command ends.
     * @param speed the desired RPM of the shooter motors
     * @param time the amount of time the command will last
     * @return the command
     */
    public static Command spinUpShooter(double speed, double time, ShooterSubsystem shooter) {
        return shooter.startShooter(speed).andThen(Commands.waitSeconds(time));
    }

    // +++ Intake +++

    /**
     * Moves the intake down, starts the intake motors, and starts the transfer.
     * Once a note is detected in the transfer, the command will end and everything stops.
     * @return the command
     */
    public static Command intakeNoteCommand(IntakeSubsystem intake, TransferSubsystem transfer) {
        return (transfer.intakeNoteCommand()
            .alongWith(intake.deployIntake()))
            .andThen(intake.stopRollers()
            .andThen(intake.moveIntakeUp()));
    }

    /**
     * Starts the intake rollers. autonStopIntake() must be called after
     * to stop the rollers.
     * @return the command
     */
    public static Command autoIntakeNoteCommand(IntakeSubsystem intake, TransferSubsystem transfer) {
        return intake.moveIntakeDown()
            .andThen(intake.startRollers())
            .alongWith(transfer.intakeNoteCommand());
    }

    /**
     * Stops the intake rollers if they are running and moves the intake
     * back up.
     * @return the command
     */
    public static Command autoStopIntakeCommand(IntakeSubsystem intake, TransferSubsystem transfer) {
        return
            intake.stopRollers()
            .andThen(transfer.transferForceStop())
            .andThen(intake.moveIntakeUp());
    }

    // +++ Arm +++

    /**
     * Moves a stored note into the arm
     * @return the command
     */
    public static Command moveToShooterCommand(ArmSubsystem arm, ShooterSubsystem shooter, TransferSubsystem transfer) {
        return shooter.setPivotTarget(ShooterSubsystem.Constants.armStage1Angle)
            .andThen(Commands.waitSeconds(0.1))
            .andThen(shooter.startShooter(0.05))
            .andThen(transfer.feedArmCommand())
            .andThen(shooter.stopShooter())
            .andThen(Commands.waitSeconds(0.2))
            .andThen(shooter.stowPivot());
    }

    public static Command shootIntoArmCommand(ArmSubsystem arm, ShooterSubsystem shooter) {
        return shooter.setPivotTarget(ShooterSubsystem.Constants.armStage2Angle)
            .andThen(Commands.waitSeconds(0.1))
            .andThen(arm.runRollers(-0.4))
            .andThen(Commands.waitSeconds(0.32))
            .andThen(shooter.startShooter(0.3))
            .andThen(Commands.waitSeconds(0.18))
            .andThen(arm.stopRollers())
            .andThen(shooter.stopShooter());
    }

    // +++ Utility +++

    /**
     * Stops the motors of both the intake and the transfer, and stows the intake.
     * @return the command
     */
    public static Command stopAllSubsystems(IntakeSubsystem intake, TransferSubsystem transfer, ShooterSubsystem shooter, ArmSubsystem trapper) {
        return transfer.transferForceStop()
            .andThen(intake.stopRollers())
            .andThen(intake.moveIntakeUp())
            .andThen(shooter.stopShooter())
            .andThen(trapper.stopRollers());
    }

    public static Command climberUpCommand(ClimberSubsystem climber) {
        return climber.runClimber(0.75); // Was at 0.6
    }

    public static Command climberDownCommand(ClimberSubsystem climber) {
        return climber.runClimber(-0.5);
    }

    public static Command climberHangCommand(ClimberSubsystem climber) {
        return climber.runClimber(-0.1);
    }
}
