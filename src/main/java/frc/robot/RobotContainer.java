package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private ClimberSubsystem climberSubsystem;
    private TransferSubsystem transfer;
    private TrapperSubsystem trapper;

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
        intake = new IntakeSubsystem();
        shooter = new ShooterSubsystem();
        climberSubsystem = new ClimberSubsystem();
        transfer = new TransferSubsystem();
        trapper = new TrapperSubsystem();

        auto = new Autonomous(driveSubsystem, shooter);
        teleop = new Teleop(driveSubsystem, driverController, operatorController);

        NamedCommands.registerCommand("speaker-shoot", auto.enableRotationCommand()
            .andThen(() -> driveSubsystem.setAllModuleDriveRawPower(0))
            .andThen(new WaitCommand(0.5))
            .andThen(CommandSequences.transferToShooterCommand(intake, transfer, shooter, trapper))
            .andThen(auto.disableRotationCommand()));
        NamedCommands.registerCommand("intake-down", 
            CommandSequences.intakeAndTransfer(intake, transfer).withTimeout(3));
        NamedCommands.registerCommand("stop-all", 
            CommandSequences.stopAllSubsystems(intake, transfer, shooter, trapper));
        
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
        teleop.init(shooter, intake, transfer, trapper, climberSubsystem);
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
