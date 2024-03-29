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
            drivetrain::getPose,
            drivetrain::resetOdometry,
            drivetrain::getSpeeds, // Robot Relative
            drivetrain::driveTranslationRotationRaw, // Robot Relative
            new HolonomicPathFollowerConfig(
                new PIDConstants(4.0, 0, 0), // Translation
                new PIDConstants(3.5, 0, 0.1), // Rotation
                SwerveDrivetrain.Constants.driveMaxSpeed,
                0.26,
                new ReplanningConfig()
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            drivetrain
        );
    }
}
