package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    public static boolean shouldFlip;
    public static double calculatedShooterSpeed = 0.9;

    /**
     * Uses the limelight and AprilTags to just point towards the speaker, from
     * wherever the robot is on the field. The shooter will also aim for a shot.
     */
    public static Command pointAndAimCommand(SwerveDrivetrain driveSubsystem, ShooterSubsystem shooter, AprilTagVision vision) {
        return new RunCommand(
            () -> {

                DriverStation.getAlliance().ifPresent((alliance) -> {
                    shouldFlip = (alliance == Alliance.Blue);
                });

                // Sets where it should point (field space coords)
                double targetX = 8.308467 * (shouldFlip ? -1 : 1);
                double targetY = 1.42593; // 1.42593
                // Z is vertical in this case
                double targetZ = 1.451102;
            
                if (vision.hasTarget()) vision.updateValues();

                // Gets the position of the robot from the limelight data
                Pose2d pose = driveSubsystem.getPose();
                double shooterZ = 0.2;

                // Calculate necessary angles and distances
                double distance = Math.sqrt(Math.pow(targetX - pose.getX(), 2) + Math.pow(targetY - pose.getY(), 2));
                double desiredAngle = Math.atan2(targetY - pose.getY(), targetX - pose.getX()) + (shouldFlip ? Math.PI : 0);
                double desiredVerticalAngle = Units.radiansToDegrees(Math.atan2(targetZ - shooterZ, distance));

                Rotation2d currentAngle = pose.getRotation();
                Rotation2d targetAngle = Rotation2d.fromRadians(desiredAngle); // Desired angle

                double angleOffset = 26; // as this value increases, the angle gets higher
                double shootAngle = (-desiredVerticalAngle + 90) - angleOffset;

                double kP = 0.025; // The amount of force it turns to the target with
                double error = targetAngle.minus(currentAngle).getDegrees(); // Calculate error
                if (error > 20) error = 20;
                if (error < -20) error = -20;

                if (shootAngle < 27) calculatedShooterSpeed = 0.6;
                else calculatedShooterSpeed = 0.9;

                // Actually drive the swerve base and set the shooter target
                shooter.setPivotTargetRaw(shootAngle);
                driveSubsystem.driveTranslationRotationRaw(
                    new ChassisSpeeds(0, 0, error * kP)
                );

                // Post debug values
                // SmartDashboard.putNumber("Desired angle", targetAngle.getDegrees());
                // SmartDashboard.putNumber("Current angle", currentAngle.getDegrees());
                // SmartDashboard.putNumber("Distance", distance);
                SmartDashboard.putNumber("Target Shoot Angle", shootAngle);
            },
            driveSubsystem
        ).withTimeout(0.8).andThen(() -> driveSubsystem.setAllModuleDriveRawPower(0));
    }

    /**
     * Uses the limelight and AprilTags to point towards the speaker and
     * shoot a stored note once centered. This command takes 1.3 seconds to complete.
     * @return the command
     */
    public static Command speakerAimAndShootCommand(SwerveDrivetrain driveSubsystem, AprilTagVision vision,
        TransferSubsystem transfer, ShooterSubsystem shooter) {
        return pointAndAimCommand(driveSubsystem, shooter, vision)
            .alongWith(shooter.startAndRun(calculatedShooterSpeed, 0.5))
            .andThen(startShooterAndTransfer(calculatedShooterSpeed, shooter, transfer).withTimeout(0.5))
            .andThen(stopShooterAndTransfer(shooter, transfer));
    }

    
    public static Command rawShootCommand(double speed, TransferSubsystem transfer, ShooterSubsystem shooter) {
        return shooter.startAndRun(calculatedShooterSpeed, 0.5)
            .andThen(startShooterAndTransfer(speed, shooter, transfer).withTimeout(0.5))
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
            intake.moveIntakeDown()
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
            .andThen(intake.moveIntakeUp());
    }
    
    /**
     * TODO
     * @return the command
     */
    public static Command raiseClimberCommand(ClimberSubsystem climb, IntakeSubsystem intake) {
        return intake.moveIntakeDown().andThen(climb.climberUpCommand());
    }

    /**
     * TODO
     * @return the command
     */
    public static Command lowerClimberCommand(ClimberSubsystem climb, IntakeSubsystem intake) {
        return intake.moveIntakeDown().andThen(climb.climberDownCommand());
    }

    /**
     * Starts both the shooter and transfer. You probably shouldn't call this
     * until the shooter motors are spun up first.
     * @return the command
     */
    public static Command startShooterAndTransfer(double speed, ShooterSubsystem shooter, TransferSubsystem transfer) {
        return shooter.startShooter(speed)
            .alongWith(transfer.removeForceStop().andThen(transfer.feedShooterCommand()));
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
        return (transfer.intakeNoteCommand(() -> false)
            .alongWith(intake.deployIntake()))
            .andThen(intake.stopRollers()
            .andThen(intake.moveIntakeUp()));
    }

    public static Command moveToTrapper(TrapperSubsystem trapper, ShooterSubsystem shooter, TransferSubsystem transfer) {
        return trapper.stage1()
            .andThen(timingCommand(0.5))
            .andThen(trapper.runMotor())
            .andThen(rawShootCommand(0.1, transfer, shooter))
            // .andThen(timingCommand(0.2))
            .andThen(shooter.stopShooter())
            .andThen(trapper.stopMotor());
    }

    /**
     * Stops the motors of both the intake and the transfer, and stows the intake.
     * @return the command
     */
    public static Command stopAllSubsystems(IntakeSubsystem intake, TransferSubsystem transfer, ShooterSubsystem shooter) {
        return transfer.transferForceStop().andThen(intake.stopRollers())
            .andThen(intake.moveIntakeUp()).andThen(shooter.stopShooter());
    }

    public static Command timingCommand(double seconds) {
        return new RunCommand(() -> {}).withTimeout(seconds);
    }
}
