package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Test Auto");
    }

    private void configureBindings() {
        // Example usage of this method:
        // driverController.b().whileTrue(exampleCommand());
    }
}
