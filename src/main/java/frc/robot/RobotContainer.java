package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.vision.AprilTagVision;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Subsystems
    private final SwerveDrivetrain driveSubsystem = new SwerveDrivetrain();

    // Misc stuff
    private final AprilTagVision vision = new AprilTagVision();
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    // OpModes
    private final Teleop teleop = new Teleop(driveSubsystem, driverController, operatorController);
    private final Autonomous auto = new Autonomous(driveSubsystem, autoChooser);

    public RobotContainer() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void runTeleop() {
        // Cancel autonomous in case it's still running for whatever reason
        auto.cancelAutonomous();
        teleop.init();
    }

    public void runAutonomous() {
        auto.init();
    }
}
