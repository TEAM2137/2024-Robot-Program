package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.vision.AprilTagVision;

/**
 * Static class containing all of the necessary command sequences for auton/teleop
 */
public class CommandSequences {
    /**
     * Uses the limelight and AprilTags to just point towards the speaker, from
     * wherever the robot is on the field. The shooter will also aim for a shot.
     */
    public static Command pointAndAimCommand(SwerveDrivetrain driveSubsystem, ShooterSubsystem shooter, AprilTagVision vision) {
        return new RunCommand(
            () -> {
                // Sets where it should point (field space coords)
                double targetX = -8.308467;

                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                    targetX *= -1; // Flip the target position based if we're on red alliance
                }

                double targetY = 1.442593;
                // Z is vertical in this case
                double targetZ = 1.451102;
            
                if (vision.hasTarget()) vision.updateValues();

                // Gets the position of the robot from the limelight data
                double robotX = vision.getX();
                double robotY = vision.getY();
                double robotZ = 0;

                // Calculate necessary angles and distances
                double distance = Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2));
                double desiredAngle = Math.atan2(targetY - robotY, targetX - robotX);
                double desiredVerticalAngle = Units.radiansToDegrees(Math.atan2(targetZ - robotZ, distance));

                Rotation2d currentAngle = driveSubsystem.getRobotAngle();
                Rotation2d targetAngle = Rotation2d.fromRadians(desiredAngle); // Desired angle

                double angleOffset = 6; // 6 degrees
                double shootAngle = (-desiredVerticalAngle + 90) - (20 + angleOffset);

                double kP = 0.03; // The amount  of force it turns to the target with
                double error = targetAngle.minus(currentAngle).getDegrees(); // Calculate error
                if (error > 20) error = 20;
                if (error < -20) error = -20;

                // Actually drive the swerve base and set the shooter target
                shooter.setPivotTargetRaw(shootAngle);
                driveSubsystem.driveTranslationRotationRaw(
                    new ChassisSpeeds(0, 0, error * kP)
                );

                // Post debug values
                SmartDashboard.putNumber("Desired angle", targetAngle.getDegrees());
                SmartDashboard.putNumber("Current angle", currentAngle.getDegrees());
                SmartDashboard.putNumber("Distance", distance);
                SmartDashboard.putNumber("Target Shoot Angle", shootAngle);
            },
            driveSubsystem
        ).withTimeout(1.0).andThen(() -> driveSubsystem.setAllModuleDriveRawPower(0));
    }

    /**
     * Uses the limelight and AprilTags to point towards the speaker and
     * shoot a stored note once centered. This command takes 2.5 seconds to complete.
     * @return the command
     */
    public static Command speakerAimAndShootCommand(SwerveDrivetrain driveSubsystem, AprilTagVision vision,
        TransferSubsystem transfer, ShooterSubsystem shooter) {
        return pointAndAimCommand(driveSubsystem, shooter, vision)
            .andThen(spinUpShooter(0.75, 0.9, shooter))
            .andThen(startShooterAndTransfer(0.75, shooter, transfer).withTimeout(0.6))
            .andThen(stopShooterAndTransfer(shooter, transfer));
    }

    /**
     * Starts the shooter motors for a specified amount of time. The 
     * motors will not stop when the command ends.
     * @param speed the desired RPM of the shooter motors
     * @param time the amount of time the command will last
     * @return the command
     */
    public static Command spinUpShooter(double speed, double time, ShooterSubsystem shooter) {
        return shooter.startShooter(speed).andThen(new RunCommand(() -> {})).withTimeout(time);
    }

    /**
     * Starts the intake rollers. autonStopIntake() must be called after
     * to stop the rollers.
     * @return the command
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
     * @return the command
     */
    public static Command autonStopIntake(IntakeSubsystem intake, TransferSubsystem transfer) {
        return
            intake.stopRollers()
            .andThen(transfer.transferForceStop())
            .andThen(intake.moveIntakeUp(0.3));
    }
    
    /**
     * TODO
     * @return the command
     */
    public static Command raiseClimberCommand(ClimberSubsystem climb, IntakeSubsystem intake) {
        return intake.moveIntakeDown(0.3).andThen(climb.climberUpCommand());
    }

    /**
     * TODO
     * @return the command
     */
    public static Command lowerClimberCommand(ClimberSubsystem climb, IntakeSubsystem intake) {
        return intake.moveIntakeDown(0.3).andThen(climb.climberDownCommand());
    }

    /**
     * Starts both the shooter and transfer. You probably shouldn't call this
     * until the shooter motors are spun up first.
     * @return the command
     */
    public static Command startShooterAndTransfer(double speed, ShooterSubsystem shooter, TransferSubsystem transfer) {
        return shooter.startShooter(speed)
            .alongWith(transfer.unlockTransfer().andThen(transfer.feedShooterCommand()));
    }

    /**
     * Force stops both the transfer and the shooter motors 
     * @return the command
     */
    public static Command stopShooterAndTransfer(ShooterSubsystem shooter, TransferSubsystem transfer) {
        return shooter.stopShooter().alongWith(transfer.transferForceStop());
    }

    /**
     * Moves the intake down, starts the intake motors, and starts the transfer.
     * Once a note is detected in the transfer, the command will end and everything stops.
     * @return the command
     */
    public static Command intakeAndTransfer(IntakeSubsystem intake, TransferSubsystem transfer) {
        return transfer.intakeNoteCommand(() -> false).alongWith(intake.deployIntake(0.3))
            .andThen(intake.stopRollers()).andThen(intake.moveIntakeUp(0.3));
    }

    /**
     * Stops the motors of both the intake and the transfer, and stows the intake.
     * @return the command
     */
    public static Command stopIntakeAndTransfer(IntakeSubsystem intake, TransferSubsystem transfer) {
        return transfer.transferForceStop().andThen(intake.stopRollers())
            .andThen(intake.moveIntakeUp(0.3));
    }
}
