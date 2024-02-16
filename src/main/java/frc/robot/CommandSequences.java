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
    public static Command pointToSpeakerCommand(SwerveDrivetrain driveSubsystem, ShooterSubsystem shooter, AprilTagVision vision) {
        return new RunCommand(
            () -> {
                // Sets where it should point (field space coords)
                double targetX = 8.308467;
                double targetY = 1.442593;
                // Z is vertical in this case
                double targetZ = 1.451102;
            
                if (vision.hasTarget()) vision.updateValues();

                // Gets the position of the robot from the limelight data
                double robotX = vision.getX();
                double robotY = vision.getY();
                // Height of the shooter, this is kind of a guess
                double robotZ = 0;

                // Calculate necessary angles/distances to point to the speaker
                double distance = Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2));
                double desiredAngle = Math.atan2(targetY - robotY, targetX - robotX);
                double desiredVerticalAngle = Units.radiansToDegrees(Math.atan2(targetZ - robotZ, distance));

                Rotation2d currentAngle = driveSubsystem.getRobotAngle();
                Rotation2d targetAngle = Rotation2d.fromRadians(desiredAngle); // Desired angle
                
                SmartDashboard.putNumber("Desired angle", targetAngle.getDegrees());
                SmartDashboard.putNumber("Current angle", currentAngle.getDegrees());
                SmartDashboard.putNumber("Distance", distance);

                double angleOffset = 6; // 6 degrees

                double shootAngle = (-desiredVerticalAngle + 90) - (20 + angleOffset);

                SmartDashboard.putNumber("Target Shoot Angle", shootAngle);

                double kP = 0.03; // The amount  of force it turns to the target with
                double error = targetAngle.minus(currentAngle).getDegrees(); // Calculate error
                if (error > 20) error = 20;
                if (error < -20) error = -20;

                // Actually drive the swerve base and set the shooter target
                shooter.setPivotTargetRaw(shootAngle);
                driveSubsystem.driveTranslationRotationRaw(
                    new ChassisSpeeds(0, 0, error * kP)
                );
            },
            driveSubsystem
        ).withTimeout(1.0).andThen(() -> driveSubsystem.setAllModuleDriveRawPower(0));
    }

    /**
     * Uses the limelight and AprilTags to point towards the speaker and
     * shoot a stored note once centered. Has a 2 second timeout.
     */
    public static Command speakerAimAndShootCommand(SwerveDrivetrain driveSubsystem, AprilTagVision vision,
        TransferSubsystem transfer, ShooterSubsystem shooter) {
        return pointToSpeakerCommand(driveSubsystem, shooter, vision)
            .andThen(spinUpShooter(0.75, 1.0, shooter))
            .andThen(startShooterAndTransfer(0.75, shooter, transfer).withTimeout(0.5))
            .andThen(stopShooterAndTransfer(shooter, transfer));
    }

    public static Command spinUpShooter(double speed, double time, ShooterSubsystem shooter) {
        return shooter.startShooter(speed).andThen(new RunCommand(() -> {})).withTimeout(time);
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
            intake.moveIntakeDown(0.3) // Lower intake
            .andThen(intake.startRollers()) // Start intake
            .alongWith(transfer.intakeNoteCommand(earlyStop) // Start transfer
            .andThen(intake.stopRollers())) // When transfer finishes stop the intake
            .andThen(intake.moveIntakeUp(0.3)) // Raise intake again
            .withTimeout(timeout);
    }

    /**
     * Starts the intake rollers. autonStopIntake() must be called after
     * to stop the rollers.
     */
    public static Command autonStartIntake(IntakeSubsystem intake, TransferSubsystem transfer) {
        return
            intake.moveIntakeDown(0.3)
            .andThen(intake.startRollers())
            .alongWith(transfer.intakeNoteCommand(() -> false));
    }

    /**
     * Stops the intake rollers if they are running and moves the intake
     * back up.
     */
    public static Command autonStopIntake(IntakeSubsystem intake, TransferSubsystem transfer) {
        return
            intake.stopRollers()
            .andThen(transfer.transferForceStop())
            .andThen(intake.moveIntakeUp(0.3));
    }
    
    public static Command raiseClimberCommand(ClimberSubsystem climb, IntakeSubsystem intake) {
        return
            intake.moveIntakeDown(0.3)
            .andThen(climb.climberUpCommand());
    }

    public static Command lowerClimberCommand(ClimberSubsystem climb, IntakeSubsystem intake) {
        return
            intake.moveIntakeDown(0.3)
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

    public static Command startShooterAndTransfer(double speed, ShooterSubsystem shooter, TransferSubsystem transfer) {
        return shooter.startShooter(speed)
            .alongWith(transfer.unlockTransfer().andThen(transfer.feedShooterCommand()));
    }

    public static Command stopShooterAndTransfer(ShooterSubsystem shooter, TransferSubsystem transfer) {
        return shooter.stopShooter()
            .alongWith(transfer.transferForceStop());
    }

    public static Command intakeAndTransfer(double speed, IntakeSubsystem intake, TransferSubsystem transfer) {
        return transfer.intakeNoteCommand(() -> false).alongWith(intake.deployIntake(speed))
            .andThen(intake.stopRollers()).andThen(intake.moveIntakeUp(0.3));
    }

    public static Command stopIntakeAndTransfer(double d, IntakeSubsystem intake, TransferSubsystem transfer) {
        return transfer.transferForceStop().andThen(intake.stopRollers())
            .andThen(intake.moveIntakeUp(0.25));
    }
}
