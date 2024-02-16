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
    private CommandXboxController driverController;
    private CommandXboxController operatorController;

    // Subsystems
    private SwerveDrivetrain driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private ClimberSubsystem climberSubsystem;
    private TransferSubsystem transferSubsystem;
    // TODO: private TrapperSubsystem trapperSubsystem = new TrapperSubsystem();

    // Misc stuff
    private final AprilTagVision vision = new AprilTagVision();
    private final SendableChooser<Command> autoChooser;

    // OpModes
    public final Teleop teleop;
    public final Autonomous auto;

    // Auton Test stuff
    //private final TestingSubsystem testSubsystem = new TestingSubsystem();

    // Initialize all subsystems and stuff here.
    public RobotContainer() {
        inst = this;

        // Init controllers
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);

        // Initialize subsystems
        driveSubsystem = new SwerveDrivetrain();
        intakeSubsystem = new IntakeSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        climberSubsystem = new ClimberSubsystem();
        transferSubsystem = new TransferSubsystem();

        auto = new Autonomous(driveSubsystem);
        teleop = new Teleop(driveSubsystem, driverController, operatorController, vision);

        //NamedCommands.registerCommand("testMotorOn", testSubsystem.testMotorOn());
        //NamedCommands.registerCommand("testMotorOff", testSubsystem.testMotorOff());
        //NamedCommands.registerCommand("aimAndShootAtSpeaker", 
        //    CommandSequences.speakerAimAndShootCommand(driveSubsystem, vision, transferSubsystem, trapperSubsystem, shooterSubsystem));
        NamedCommands.registerCommand("pointToSpeaker", 
            CommandSequences.pointToSpeakerCommand(driveSubsystem, shooterSubsystem, vision));
        //NamedCommands.registerCommand("startIntake", CommandSequences.startIntakeCommand(intakeSubsystem, transferSubsystem, () -> false, 5.0));

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
