package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final SwerveDrivetrain driveSubsystem = new SwerveDrivetrain();
    public static final boolean disableSwerve = false;

    //private final AprilTagVision vision = new AprilTagVision();

    private final Teleop teleop = new Teleop(driveSubsystem, driverController, operatorController);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putBoolean("Save Offsets", false);
        //driveSubsystem.displayCurrentOffsets();
        //driveSubsystem.loadOffsets();
    }

    public void runTeleop() {
        teleop.teleopInit();
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Test Auto");
    }

    private void configureBindings() {
        // Example usage of this method:
        // driverController.b().whileTrue(exampleCommand());
    }

    public void autonomousInit() {
        driveSubsystem.resetGyro();
        driveSubsystem.resetDriveDistances();
    }
}
