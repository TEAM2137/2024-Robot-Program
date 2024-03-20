package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Autonomous {
    private SwerveDrivetrain drivetrain;
    private SendableChooser<Command> autoChooser;

    private Command autonomousCommand;
    private Command autoAimCommand;
    private boolean enableRotation;

    public Autonomous(SwerveDrivetrain drivetrain, ShooterSubsystem shooter) {
        this.drivetrain = drivetrain;

        autoAimCommand = Commands.run(() -> {
            double rot = RobotContainer.getInstance().teleop.targetSpeakerUpdate(shooter);
            if (enableRotation) {
                drivetrain.driveTranslationRotationRaw(
                    new ChassisSpeeds(0, 0, rot));
            }
        });
    }

    public void init() {
        // Init autonomous stuff
        drivetrain.resetOdometry(new Pose2d());
        
        // Run auton command
        autonomousCommand = autoChooser.getSelected();

        CommandScheduler.getInstance().schedule(autonomousCommand.alongWith(autoAimCommand)
            .until(() -> DriverStation.isDisabled() || DriverStation.isTeleop()));
    }

    public Command enableRotationCommand() {
        return Commands.runOnce(() -> enableRotation = true);
    }

    public Command disableRotationCommand() {
        return Commands.runOnce(() -> enableRotation = false);
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
                new PIDConstants(5.0, 0.0, 0.01), // Translation
                new PIDConstants(5.0, 0.0, 0.01), // Rotation
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
