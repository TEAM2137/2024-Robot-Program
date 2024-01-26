package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Autonomous {

    private SwerveDrivetrain driveSubsystem;
    private SendableChooser<Command> autoChooser;

    private Command autonomousCommand;

    public Autonomous(SwerveDrivetrain driveSubsystem, SendableChooser<Command> autoChooser) {
        this.driveSubsystem = driveSubsystem;
        this.autoChooser = autoChooser;
    }

    public void init() {
        // Init autonomous stuff
        driveSubsystem.resetGyro();
        driveSubsystem.resetDriveDistances();
        autonomousCommand = autoChooser.getSelected();
        CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    public void cancelAutonomous() {
        autonomousCommand.cancel();
    }
}
