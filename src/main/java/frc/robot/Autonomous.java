package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    private boolean pathTargeting;
    private boolean pathEndTargeting;
    private Optional<Rotation2d> targetRotation;

    public Autonomous(SwerveDrivetrain drivetrain, ShooterSubsystem shooter) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
    }

    public void init() {        
        autonomousCommand = autoChooser.getSelected();
        drivetrain.resetModuleAngles();

        // Set pathplanner rotation override method
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

        // Schedule the actual command
        CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    public void periodic() {
        double rot = RobotContainer.getInstance().teleop.targetUpdate(shooter, ShotLocation.SPEAKER);
        if (pathTargeting) {
            targetRotation = Optional.of(drivetrain.getRotation().plus(new Rotation2d(rot / 2)));
        }
        if (pathEndTargeting) {
            drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(0, 0, rot));
        }
    }
    
    public Optional<Rotation2d> getRotationTargetOverride() {
        if (pathTargeting) return targetRotation;
        else return Optional.empty();
    }

    public void configure() {
        AutoBuilder.configureHolonomic(
            drivetrain::getPose,
            drivetrain::resetOdometry,
            drivetrain::getSpeeds, // Robot Relative
            drivetrain::driveTranslationRotationRaw, // Robot Relative
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), // Translation
                new PIDConstants(5.0, 0, 0), // Rotation
                SwerveDrivetrain.Constants.driveMaxSpeed,
                0.26,
                new ReplanningConfig()
            ),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            drivetrain
        );
    }

    public Command enableTargetingCommand() {
        return Commands.runOnce(() -> pathTargeting = true);
    }

    public Command disableTargetingCommand() {
        return Commands.runOnce(() -> { pathTargeting = false; pathEndTargeting = false; });
    }

    public boolean isPathTargeting() { return pathTargeting; }

    public void setAutoChooser(SendableChooser<Command> autoChooser) { this.autoChooser = autoChooser; }

    public void cancelAutonomous() {
        if (autonomousCommand != null && !autonomousCommand.isFinished()) autonomousCommand.cancel();
    }

    public Command pathEndAimCommand() {
       return Commands.runOnce(() -> pathEndTargeting = true);
    }
}
