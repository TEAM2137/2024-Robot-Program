package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.vision.AprilTagVision;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Main class that contains all the robot subsystems, controllers, opModes, etc.
 */
public class RobotContainer {
    
    private static RobotContainer inst;

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Subsystems
    private final SwerveDrivetrain driveSubsystem = new SwerveDrivetrain();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final TransferSubsystem transferSubsystem = new TransferSubsystem();
    private final TrapperSubsystem trapperSubsystem = new TrapperSubsystem();

    // Misc stuff
    private final AprilTagVision vision = new AprilTagVision();
    private final SendableChooser<Command> autoChooser;

    // OpModes
    public final Teleop teleop;
    public final Autonomous auto;

    // Auton Test stuff
    private final TestingSubsystem testSubsystem = new TestingSubsystem();

    // Initialize all subsystems and stuff here.
    public RobotContainer() {
        inst = this;

        auto = new Autonomous(driveSubsystem);
        teleop = new Teleop(driveSubsystem, driverController, operatorController);

        NamedCommands.registerCommand("testMotorOn", testSubsystem.testMotorOn());
        NamedCommands.registerCommand("testMotorOff", testSubsystem.testMotorOff());
        NamedCommands.registerCommand("aimAndShootAtSpeaker", 
            CommandSequences.speakerAimAndShootCommand(driveSubsystem, vision, transferSubsystem, trapperSubsystem, shooterSubsystem));
        NamedCommands.registerCommand("pointToSpeaker", 
            CommandSequences.pointToSpeakerCommand(driveSubsystem, vision));
        NamedCommands.registerCommand("startIntake", CommandSequences.startIntakeCommand(intakeSubsystem, transferSubsystem, () -> false, 5.0));

        autoChooser = AutoBuilder.buildAutoChooser();
        auto.setAutoChooser(autoChooser);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Called when teleop is enabled
     */
    public void runTeleop() {
        // Cancel autonomous in case it's still running for whatever reason
        auto.cancelAutonomous();
        teleop.init(shooterSubsystem, intakeSubsystem, transferSubsystem);
    }

    /**
     * Called when autonomous is enabled
     */
    public void runAutonomous() {
        auto.init();
    }

    /**
     * Use this to get the main instance of the container when necessary
     */
    public static RobotContainer getInstance() { return inst; }
}
