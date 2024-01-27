package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TestingSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.CanIDs;
import frc.robot.vision.AprilTagVision;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Subsystems
    private final SwerveDrivetrain driveSubsystem = new SwerveDrivetrain();
    // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    // private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    // private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    // private final TransferSubsystem TransferSubsystem = new TransferSubsystem();

    // Misc stuff
    private final AprilTagVision vision = new AprilTagVision();
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    // OpModes
    private final Teleop teleop = new Teleop(driveSubsystem, driverController, operatorController);
    private final Autonomous auto = new Autonomous(driveSubsystem, autoChooser);

    // Auton Test stuff
    private final TestingSubsystem testSubsystem = new TestingSubsystem();

    public RobotContainer() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        NamedCommands.registerCommand("testMotorOn", testSubsystem.testMotorOn());
        NamedCommands.registerCommand("testMotorOff", testSubsystem.testMotorOff());
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
