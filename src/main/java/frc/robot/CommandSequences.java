package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TrapperSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.vision.AprilTagVision;

/**
 * Static class containing all of the necessary command sequences for auton/teleop
 */
public class CommandSequences {
    /**
     * Uses the limelight and AprilTags to just point towards the speaker, from
     * wherever the robot is on the field.
     */
    public static Command pointToSpeakerCommand(SwerveDrivetrain driveSubsystem, AprilTagVision vision) {
        return new RunCommand(
            () -> {
                // Sets where it should point (field space coords)
                double targetX = -8.308467;
                double targetY = 1.442593;
            
                if (vision.hasTarget()) vision.updateValues();

                // Gets the position of the robot from the limelight data
                double robotX = vision.getX();
                double robotY = vision.getY();

                // Calculate necessary angle to point to the speaker
                double desiredAngle = Math.atan2(targetY - robotY, targetX - robotX);

                SmartDashboard.putNumber("Desired angle", Units.radiansToDegrees(desiredAngle));
                SmartDashboard.putNumber("Current angle", driveSubsystem.getRobotAngle().getDegrees());

                Rotation2d currentAngle = driveSubsystem.getRobotAngle().plus(Rotation2d.fromDegrees(180));
                Rotation2d targetAngle = Rotation2d.fromRadians(desiredAngle); // Desired angle

                double kP = 0.03; // The amount of force it turns to the target with
                double error = targetAngle.minus(currentAngle).getDegrees(); // Calculate error
                if (error > 10) error = 10;
                if (error < -10) error = -10;

                // Actually drive the swerve base
                driveSubsystem.driveTranslationRotationRaw(
                    new ChassisSpeeds(0, 0, error * kP)
                );
            },
            driveSubsystem
        ).withTimeout(1.5);
    }

    /**
     * Uses the limelight and AprilTags to point towards the speaker and
     * shoot a stored note once centered. Has a 2 second timeout.
     */
    public static Command speakerAimAndShootCommand(SwerveDrivetrain driveSubsystem, AprilTagVision vision,
        TransferSubsystem transfer, TrapperSubsystem trapper, ShooterSubsystem shooter) {
        return pointToSpeakerCommand(driveSubsystem, vision)
        .andThen(shooter.runShooter(1.0, .7))
        .alongWith(transferToShooter(transfer, trapper))
        .withTimeout(2.0);
    }

    /**
     * Starts the intake rollers until either there is a note in the intake
     * or the early stop condition is met.
     * 
     * @param timeout the amount of time the intake will move up for.
     * @param earlyStop BooleanSupplier to stop the command early for whatever reason
     */
    public static Command startIntakeCommand(IntakeSubsystem intake, TransferSubsystem transfer,
        BooleanSupplier earlyStop, double timeout) {
        return 
            intake.moveIntakeDown() // Lower intake
            .andThen(intake.startMotors()) // Start intake
            .alongWith(transfer.intakeCommand(earlyStop) // Start transfer
            .andThen(intake.pleaseStop())) // When transfer finishes stop the intake
            .andThen(intake.moveIntakeUp()) // Raise intake again
            .withTimeout(timeout);
    }

    /**
     * Starts the intake rollers. autonStopIntake() must be called after
     * to stop the rollers.
     */
    public static Command autonStartIntake(IntakeSubsystem intake, TransferSubsystem transfer) {
        return
            intake.moveIntakeDown()
            .andThen(intake.startMotors())
            .alongWith(transfer.intakeCommand(() -> false));
    }

    /**
     * Stops the intake rollers if they are running and moves the intake
     * back up.
     */
    public static Command autonStopIntake(IntakeSubsystem intake, TransferSubsystem transfer) {
        return
            intake.pleaseStop()
            .andThen(transfer.shutoffCommand())
            .andThen(intake.moveIntakeUp());
    }
    
    public static Command raiseClimberCommand(ClimberSubsystem climb, IntakeSubsystem intake) {
        return
            intake.moveIntakeDown()
            .andThen(climb.climberUpCommand());
    }

    public static Command lowerClimberCommand(ClimberSubsystem climb, IntakeSubsystem intake) {
        return
            intake.moveIntakeDown()
            .andThen(climb.climberDownCommand());
    }

    public static Command transferToShooter(TransferSubsystem transfer, TrapperSubsystem trapper) {
        return
            trapper.setArmTarget(30)
            .andThen(transfer.feedShooterCommand())
            .andThen(trapper.setArmTarget(0)); // Might need to break this into a separate command to reset the trapper position.
    }

    public static Command transferToTrapper(TransferSubsystem transfer, TrapperSubsystem trapper) {
        return
            transfer.feedTrapperCommand()
            .alongWith(trapper.runMotor()) // I'm thinking to transfer it completely we'll need the trapper to intake a little bit
            .andThen(trapper.stopMotor());
    }
}
