package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandSequences;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.ModuleType;
import frc.robot.util.LEDs;
import frc.robot.vision.VisionBlender;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * Main class that contains all the robot subsystems, controllers, opModes, etc.
 */
public class RobotContainer {
    
    private static RobotContainer inst;

    // Controllers
    public CommandXboxController driverController;
    private CommandXboxController operatorController;

    // Subsystems
    private SwerveDrivetrain driveSubsystem;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private ClimberSubsystem climberSubsystem;
    private TransferSubsystem transfer;
    private ArmSubsystem arm;

    // Misc stuff
    private final VisionBlender vision = new VisionBlender("limelight-a", "limelight-b");
    private final LEDs leds = new LEDs();

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
        intake = new IntakeSubsystem();
        shooter = new ShooterSubsystem();
        climberSubsystem = new ClimberSubsystem();
        transfer = new TransferSubsystem();
        arm = new ArmSubsystem();
        
        auto = new Autonomous(driveSubsystem, shooter);
        teleop = new Teleop(driveSubsystem, driverController, operatorController, leds);

        NamedCommands.registerCommand("speaker-shoot", auto.enableTargetingCommand()
            .andThen(() -> driveSubsystem.setAllModuleDriveRawPower(0))
            .andThen(CommandSequences.transferToShooterCommand(intake, transfer, shooter, arm))
            .andThen(auto.disableTargetingCommand()));

        NamedCommands.registerCommand("manual-shot", auto.disableTargetingCommand()
            .andThen(shooter.setPivotTarget(ShooterSubsystem.Constants.manualClose))
            .andThen(CommandSequences.rawShootCommand(0.7, transfer, shooter)));

        NamedCommands.registerCommand("path-end-aim", auto.pathEndAimCommand());

        NamedCommands.registerCommand("speaker-aim", auto.enableTargetingCommand());

        NamedCommands.registerCommand("intake-down", //Commands.waitSeconds(1.5));
           CommandSequences.intakeAndTransfer(intake, transfer).withTimeout(3));
            
        NamedCommands.registerCommand("stop-all", auto.disableTargetingCommand()
            .andThen(CommandSequences.stopAllSubsystems(intake, transfer, shooter, arm)));
        
        auto.configure();

        autoChooser = AutoBuilder.buildAutoChooser();
        auto.setAutoChooser(autoChooser);

        SmartDashboard.putData("Drivetrain Type", drivetrainType);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
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
        vision.resetAlliances();
        
        // Cancel autonomous in case it's still running for whatever reason
        auto.cancelAutonomous();

        // Initialize teleop
        teleop.init(shooter, intake, transfer, arm, climberSubsystem);
    }

    /**
     * Called when autonomous is enabled
     */
    public void runAutonomous() {
        vision.resetAlliances();

        // Initialize auto
        auto.init();
    }

    public void onDisabled() {
        leds.onDisabled();
        driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    /**
     * Use this to get the main instance of the container when necessary
     */
    public static RobotContainer getInstance() { return inst; }
}
