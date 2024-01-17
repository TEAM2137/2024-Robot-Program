package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final SwerveDrivetrain driveSubsystem = new SwerveDrivetrain();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void runTeleop() {
        driveSubsystem.setDefaultCommand(
            new RunCommand(
                () -> {
                    double speedX = driverController.getLeftX();
                    double speedY = driverController.getLeftY();
                    double rot = driverController.getRightX();

                    // Controller deadzones
                    speedX = Math.abs(speedX) < 0.2 ? 0 : speedX;
                    speedY = Math.abs(speedY) < 0.2 ? 0 : speedY;
                    rot = Math.abs(rot) < 0.2 ? 0 : rot;
                    
                    driveSubsystem.driveTranslationRotationRaw(
                        new ChassisSpeeds(speedX, speedY, rot * Math.PI)
                    );
                },
                driveSubsystem
        ));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Test Auto");
    }

    private void configureBindings() {
        // Example usage of this method:
        // driverController.b().whileTrue(exampleCommand());
    }
}
