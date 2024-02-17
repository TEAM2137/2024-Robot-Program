package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Autonomous {

    private SwerveDrivetrain swerve;
    private SendableChooser<Command> autoChooser;

    private Command autonomousCommand;

    public Autonomous(SwerveDrivetrain driveSubsystem) {
        this.swerve = driveSubsystem;
    }

    public void init() {
        // Init autonomous stuff
        swerve.resetGyro();
        swerve.resetDriveDistances();
        autonomousCommand = autoChooser.getSelected();
        CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    public void setAutoChooser(SendableChooser<Command> autoChooser) { this.autoChooser = autoChooser; }

    public void cancelAutonomous() {
        if (autonomousCommand != null && !autonomousCommand.isFinished())
            autonomousCommand.cancel();
    }

    public void configure() {
        // Pathplanner Initialization
        AutoBuilder.configureHolonomic(
            swerve::getPose, // Robot pose supplier
            swerve::resetOdometry, // Method to reset odometry
            swerve::getSpeeds, // ChassisSpeeds supplier, ROBOT RELATIVE
            swerve::driveTranslationRotationRaw, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig(
                new PIDConstants(2.0, 0.0, 0.01), // Translation PID constants
                new PIDConstants(6.0, 0.0, 0.01), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config.
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            swerve // Reference to the drivetrain to set requirements
        );
    }
}
