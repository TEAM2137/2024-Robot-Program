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

public class CommandSequences {

    public CommandSequences() {
        // Leaving this here in case we need to store variables between commands
    }

    // Uses the limelight and AprilTags to point towards the speaker.
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

    public static Command speakerAimAndShootCommand(
        SwerveDrivetrain driveSubsystem,
        AprilTagVision vision,
        TransferSubsystem transfer,
        TrapperSubsystem trapper,
        ShooterSubsystem shooter
        ) {
        return pointToSpeakerCommand(driveSubsystem, vision)
        .andThen(shooter.runShooter(1.0, .7))
        .alongWith(transferToShooter(transfer, trapper))
        .withTimeout(2.0);
    }

    public static Command startIntakeCommand(IntakeSubsystem intake, TransferSubsystem transfer, BooleanSupplier earlyStop, double timeout) {
        return 
            intake.MoveIntakeDown() // Lower intake
            .andThen(intake.StartMotors()) // Start intake
            .alongWith(transfer.intakeCommand(earlyStop) // Start transfer
            .andThen(intake.PleaseStop())) // When transfer finishes stop the intake
            .andThen(intake.MoveIntakeUp()) // Raise intake again
            .withTimeout(timeout);
    }

    public static Command autonStartIntake(IntakeSubsystem intake, TransferSubsystem transfer) {
        return
            intake.MoveIntakeDown()
            .andThen(intake.StartMotors())
            .alongWith(transfer.intakeCommand(() -> false));
    }

    public static Command autonStopIntake(IntakeSubsystem intake, TransferSubsystem transfer) {
        return
            intake.PleaseStop()
            .andThen(transfer.shutoffCommand())
            .andThen(intake.MoveIntakeUp());
    }

    public static Command raiseClimberCommand(ClimberSubsystem climb, IntakeSubsystem intake) {
        return
            intake.MoveIntakeDown()
            .andThen(climb.climberUpCommand());
    }

    public static Command lowerClimberCommand(ClimberSubsystem climb, IntakeSubsystem intake) {
        return
            intake.MoveIntakeDown()
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
