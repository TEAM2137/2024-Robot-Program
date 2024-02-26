package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Autonomous {
    private SwerveDrivetrain drivetrain;
    private SendableChooser<Command> autoChooser;

    private Command autonomousCommand;

    public Autonomous(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void init() {
        // Init autonomous stuff
        drivetrain.resetOdometry(new Pose2d());
        
        // Run auton command
        autonomousCommand = autoChooser.getSelected();
        CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    public void setAutoChooser(SendableChooser<Command> autoChooser) { this.autoChooser = autoChooser; }

    public void cancelAutonomous() {
        if (autonomousCommand != null && !autonomousCommand.isFinished()) autonomousCommand.cancel();
    }

    public void configure() {
        AutoBuilder.configureHolonomic(
            drivetrain::getPose, // Robot pose supplier
            drivetrain::resetOdometry, // Method to reset odometry
            drivetrain::getSpeeds, // ChassisSpeeds supplier, ROBOT RELATIVE
            drivetrain::driveTranslationRotationRaw, // Drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0.0, 0.1), // Translation
                new PIDConstants(6.0, 0.0, 0.6), // Rotation
                3.0, // Max module speed (m/s)
                0.4, // Distance from robot center to furthest module (meters)
                new ReplanningConfig()
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
            drivetrain // Reference to the drivetrain to set requirements
        );
    }
}
