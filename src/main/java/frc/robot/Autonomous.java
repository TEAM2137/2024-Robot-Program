package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Teleop.ShotLocation;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Autonomous {
    private SwerveDrivetrain drivetrain;
    private ShooterSubsystem shooter;
    private SendableChooser<Command> autoChooser;

    private Command autonomousCommand;
    private boolean enableTargeting;

    public Autonomous(SwerveDrivetrain drivetrain, ShooterSubsystem shooter) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
    }

    public void init() {
        // Init autonomous stuff
        
        // Run auton command
        autonomousCommand = autoChooser.getSelected();
        drivetrain.resetModuleAngles();

        CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    public void autonomousPeriodic() {
        double rot = RobotContainer.getInstance().teleop.targetUpdate(shooter, ShotLocation.SPEAKER);
        if (enableTargeting) {
            drivetrain.driveTranslationRotationRaw(
                new ChassisSpeeds(0, 0, rot));
        }
    }

    public Command enableRotationCommand() {
        return Commands.runOnce(() -> enableTargeting = true);
    }

    public Command disableRotationCommand() {
        return Commands.runOnce(() -> enableTargeting = false);
    }

    public boolean isTargetingEnabled() { return enableTargeting; }

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
                new PIDConstants(4.0, 0, 0), // Translation
                new PIDConstants(1.0, 0, 0), // Rotation
                4.0, // Max module speed (m/s)
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
