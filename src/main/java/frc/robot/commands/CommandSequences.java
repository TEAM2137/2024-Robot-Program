package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TrapperSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.vision.VisionBlender;

/**
 * Static class containing all of the necessary command sequences for auton/teleop
 */
public class CommandSequences {

    public static boolean shouldFlip;
    public static double calculatedShooterSpeed = 0.9;

    // +++ Shooter +++

    /**
     * Spins up the shooter motors to a specified speed and shoots with
     * whatever angle the shooter is currently at.
     * @return
     */
    public static Command rawShootCommand(double speed, TransferSubsystem transfer, ShooterSubsystem shooter) {
        return shooter.startAndRun(speed, 0.8)
            .andThen(startShooterAndTransfer(speed, shooter, transfer).withTimeout(0.8))
            .andThen(stopShooterAndTransfer(shooter, transfer));
    }

    /**
     * Shoots into the amp by aiming the shooter pivot and shooting
     * @return
     */
    public static Command ampShootCommand(double speed, TransferSubsystem transfer, ShooterSubsystem shooter) {
        return shooter.startShooter(speed)
            .andThen(Commands.waitSeconds(1.2))
            .andThen(startShooterAndTransfer(speed, shooter, transfer).withTimeout(0.8))
            .andThen(stopShooterAndTransfer(shooter, transfer));
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
     * Starts the shooter motors for a specified amount of time. The 
     * motors will not stop when the command ends.
     * @param speed the desired RPM of the shooter motors
     * @param time the amount of time the command will last
     * @return the command
     */
    public static Command spinUpShooter(double speed, double time, ShooterSubsystem shooter) {
        return shooter.startShooter(speed).andThen(Commands.waitSeconds(time));
    }

    // +++ Intake +++

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

    // +++ Arm +++

    /**
     * Moves a stored note into the arm
     * @return the command
     */
    public static Command moveToTrapper(TrapperSubsystem trapper, ShooterSubsystem shooter, TransferSubsystem transfer) {
        return trapper.stage1()
            .andThen(Commands.waitSeconds(0.5))
            .andThen(trapper.runMotor())
            .andThen(rawShootCommand(0.1, transfer, shooter))
            // .andThen(timingCommand(0.2))
            .andThen(shooter.stopShooter())
            .andThen(trapper.stopMotor());
    }

    // +++ Vision +++

    /**
     * Uses the limelight and AprilTags to point towards the speaker and
     * shoot a stored note once centered. This command takes 1.3 seconds to complete.
     * @return the command
     */
    public static Command speakerAimAndShootCommand(SwerveDrivetrain driveSubsystem, VisionBlender vision,
        TransferSubsystem transfer, ShooterSubsystem shooter) {
        return pointAndAimCommand(driveSubsystem, shooter, vision)
            .alongWith(shooter.startAndRun(calculatedShooterSpeed, 0.5))
            .andThen(startShooterAndTransfer(calculatedShooterSpeed, shooter, transfer).withTimeout(0.5))
            .andThen(stopShooterAndTransfer(shooter, transfer));
    }

    /**
     * Uses the limelight and AprilTags to just point towards the speaker, from
     * wherever the robot is on the field. The shooter will also aim for a shot.
     */
    public static Command pointAndAimCommand(SwerveDrivetrain driveSubsystem, ShooterSubsystem shooter, VisionBlender vision) {
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
            
                vision.updateValues();

                // Gets the position of the robot from the limelight data
                Pose2d pose = vision.getPose();
                double shooterZ = 0;

                // Calculate necessary angles and distances
                double distance = Math.sqrt(Math.pow(targetX - pose.getX(), 2) + Math.pow(targetY - pose.getY(), 2));
                double desiredAngle = Math.atan2(targetY - pose.getY(), targetX - pose.getX()) + (shouldFlip ? Math.PI : 0);
                double desiredVerticalAngle = Units.radiansToDegrees(Math.atan2(targetZ - shooterZ, distance));

                Rotation2d currentAngle = driveSubsystem.getRotation();
                Rotation2d targetAngle = Rotation2d.fromRadians(desiredAngle); // Desired angle

                double angleOffset = 100; // as this value decreases, the angle gets higher
                double shootAngle = (-desiredVerticalAngle + 90) + angleOffset;

                double kP = 0.025; // The amount of force it turns to the target with
                double error = -currentAngle.minus(targetAngle).getDegrees(); // Calculate error
                if (error > 20) error = 20; if (error < -20) error = -20;

                if (distance < 0.8) calculatedShooterSpeed = 0.5;
                else calculatedShooterSpeed = 0.8;

                // Actually drive the swerve base and set the shooter target
                shooter.setPivotTargetRaw(Math.max(Math.min(shootAngle, ShooterSubsystem.Constants.maxAngle),
                    ShooterSubsystem.Constants.minAngle));
                driveSubsystem.driveTranslationRotationRaw(
                    new ChassisSpeeds(0, 0, error * kP)
                );

                // Post debug values
                // SmartDashboard.putNumber("Distance", distance);
                SmartDashboard.putNumber("Raw Shoot Angle", desiredVerticalAngle);
                SmartDashboard.putNumber("Shoot Angle", shootAngle);
            },
            driveSubsystem
        ).withTimeout(1.3).andThen(() -> driveSubsystem.setAllModuleDriveRawPower(0));
    }

    // +++ Utility +++

    /**
     * Stops the motors of both the intake and the transfer, and stows the intake.
     * @return the command
     */
    public static Command stopAllSubsystems(IntakeSubsystem intake, TransferSubsystem transfer, ShooterSubsystem shooter) {
        return transfer.transferForceStop().andThen(intake.stopRollers())
            .andThen(intake.moveIntakeUp()).andThen(shooter.stopShooter());
    }

    public static Command climberUpCommand(ClimberSubsystem climber) {
        return climber.setSpeedCommand(0.5);
    }

    public static Command climberDownCommand(ClimberSubsystem climber) {
        return climber.setSpeedCommand(-0.5);
    }
}
