package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandSequences;
import frc.robot.commands.RumbleSequences;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.ModuleType;
import frc.robot.vision.AprilTagLimelight;
import frc.robot.vision.VisionBlender;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

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
    private TrapperSubsystem trapperSubsystem;

    // Misc stuff
    private final AprilTagLimelight visionA = new AprilTagLimelight("limelight-a");
    private final VisionBlender vision = new VisionBlender(visionA);

    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<ModuleType> drivetrainType;

    // OpModes
    public final Teleop teleop;
    public final Autonomous auto;

    // Initialize all subsystems and stuff here.
    public RobotContainer() {
        inst = this;

        drivetrainType = new SendableChooser<>();
        drivetrainType.setDefaultOption("Falcon", ModuleType.Falcon);
        drivetrainType.addOption("Neo", ModuleType.Neo);

        // Init controllers
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);

        // Initialize subsystems
        driveSubsystem = new SwerveDrivetrain(ModuleType.Falcon, vision);
        intakeSubsystem = new IntakeSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        climberSubsystem = new ClimberSubsystem();
        transferSubsystem = new TransferSubsystem();
        trapperSubsystem = new TrapperSubsystem();

        auto = new Autonomous(driveSubsystem);
        teleop = new Teleop(driveSubsystem, driverController, operatorController);

        NamedCommands.registerCommand("speaker-aim", 
           CommandSequences.pointAndAimCommand(driveSubsystem, shooterSubsystem, vision));
        NamedCommands.registerCommand("speaker-shoot", 
           CommandSequences.speakerAimAndShootCommand(driveSubsystem, vision, transferSubsystem, shooterSubsystem)
           .andThen(shooterSubsystem.stowPivot()));
        NamedCommands.registerCommand("intake-down", 
            CommandSequences.intakeAndTransfer(intakeSubsystem, transferSubsystem).withTimeout(3));
        
        auto.configure();

        autoChooser = AutoBuilder.buildAutoChooser();
        auto.setAutoChooser(autoChooser);

        SmartDashboard.putData("Drivetrain Type", drivetrainType);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        HttpCamera httpCamera = new HttpCamera("Limelight", "http://10.21.37.92:5800/");
        CameraServer.addCamera(httpCamera);
        
        // Starts recording to data log
        DataLogManager.start();

        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    public static String getDrivetrainCanBusName() {
        return "Drivetrain";
    }

    public static String getRioCanBusName() {
        return "rio";
    }

    /**
     * Called when teleop is enabled
     */
    public void runTeleop() {
        vision.limelights.forEach(limelight -> limelight.resetAlliance());
        
        // Cancel autonomous in case it's still running for whatever reason
        auto.cancelAutonomous();
        teleop.init(shooterSubsystem, intakeSubsystem, transferSubsystem, trapperSubsystem, climberSubsystem);
    }

    /**
     * Called when autonomous is enabled
     */
    public void runAutonomous() {
        vision.limelights.forEach(limelight -> limelight.resetAlliance());

        auto.init();
    }

    /**
     * Use this to get the main instance of the container when necessary
     */
    public static RobotContainer getInstance() { return inst; }

    public void disabledPeriodic() {
        RumbleSequences.shutOffRumble(driverController.getHID());
    }
}
