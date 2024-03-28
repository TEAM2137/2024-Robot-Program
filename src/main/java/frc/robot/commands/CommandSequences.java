package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    public static boolean isBlueAlliance;
    public static double calculatedShooterSpeed = 0.9;

    // +++ Shooter +++

    /**
     * Spins up the shooter motors to a specified speed and shoots with
     * whatever angle the shooter is currently at.
     * @return
     */
    public static Command rawShootCommand(double speed, TransferSubsystem transfer, ShooterSubsystem shooter) {
        return shooter.startAndRun(speed, 1.2)
            .andThen(startShooterAndTransfer(speed, shooter, transfer).withTimeout(0.8))
            .andThen(stopShooterAndTransfer(shooter, transfer));
    }

    /**
     * Shoots into the amp by aiming the shooter pivot and shooting
     * @return the command
     */
    public static Command ampShootCommand(double topSpeed, double bottomSpeed, TransferSubsystem transfer, ShooterSubsystem shooter) {
        return shooter.startSeperateShooters(topSpeed, bottomSpeed)
            .andThen(Commands.waitSeconds(1.2))
            .andThen(transfer.removeForceStop().andThen(transfer.feedShooterCommand()).withTimeout(0.8))
            .andThen(stopShooterAndTransfer(shooter, transfer));
    }
    // public static Command ampShootCommand(double speed, TransferSubsystem transfer, ShooterSubsystem shooter) {
    //     return shooter.startShooter(speed)
    //         .andThen(Commands.waitSeconds(1.2))
    //         .andThen(startShooterAndTransfer(speed, shooter, transfer).withTimeout(0.8))
    //         .andThen(stopShooterAndTransfer(shooter, transfer));
    // }

    public static Command transferToShooterCommand(IntakeSubsystem intake, TransferSubsystem transfer, ShooterSubsystem shooter, TrapperSubsystem trapper) {
        return transfer.feedShooterCommand().withTimeout(0.8)
            .andThen(CommandSequences.stopAllSubsystems(intake, transfer, shooter, trapper));
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
        return (transfer.intakeNoteCommand()
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
            .alongWith(transfer.intakeNoteCommand());
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
    public static Command moveToShooterForArmCommand(TrapperSubsystem trapper, ShooterSubsystem shooter, TransferSubsystem transfer) {
        return shooter.setPivotTarget(ShooterSubsystem.Constants.armStage1Angle)
            .andThen(Commands.waitSeconds(0.3))
            .andThen(shooter.startShooter(0.04))
            .andThen(transfer.feedArmCommand())
            .andThen(shooter.stopShooter())
            .andThen(Commands.waitSeconds(0.2))
            .andThen(shooter.stowPivot());
    }

    public static Command shootIntoArmCommand(TrapperSubsystem trapper, ShooterSubsystem shooter) {
        return shooter.setPivotTarget(ShooterSubsystem.Constants.armStage2Angle)
            .andThen(Commands.waitSeconds(0.1))
            .andThen(trapper.runRollers(-0.4))
            .andThen(Commands.waitSeconds(0.32))
            .andThen(shooter.startShooter(0.3))
            .andThen(Commands.waitSeconds(0.18))
            .andThen(trapper.stopRollers())
            .andThen(shooter.stopShooter());
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
                
                // Flips the aiming if the alliance is blue
                DriverStation.getAlliance().ifPresent((alliance) -> isBlueAlliance = (alliance == Alliance.Blue));

                // Sets where it should point (field space coords)
                Translation2d targetPos;
                if (isBlueAlliance) targetPos = new Translation2d(-0.4, 5.56);
                else targetPos = new Translation2d(-0.4, 2.73);
            
                vision.updateValues();

                Pose2d robotPose = driveSubsystem.getPose();
                
                double distance = Math.hypot(targetPos.getX() - robotPose.getX(), targetPos.getY() - robotPose.getY());
                double desiredAngle = Math.atan2(targetPos.getY() - robotPose.getY(), targetPos.getX() - robotPose.getX());

                Rotation2d currentAngle = robotPose.getRotation();
                Rotation2d targetAngle = Rotation2d.fromRadians(desiredAngle);

                double kP = 0.025; // The amount of force it turns to the target with
                double error = -currentAngle.minus(targetAngle).getDegrees(); // Calculate error
                if (error > 20) error = 20; if (error < -20) error = -20;

                calculatedShooterSpeed = 0.8;

                // Actually drive the swerve base and set the shooter target
                // shooter.setPivotTargetRaw(Math.max(Math.min(shootAngle, ShooterSubsystem.Constants.maxAngle),
                //     ShooterSubsystem.Constants.minAngle));

                driveSubsystem.driveTranslationRotationRaw(
                    new ChassisSpeeds(0, 0, error * kP)
                );

                // Post debug values
                SmartDashboard.putNumber("Distance", distance);
            },
            driveSubsystem
        ).withTimeout(1.3).andThen(() -> driveSubsystem.setAllModuleDriveRawPower(0));
    }

    // +++ Utility +++

    /**
     * Stops the motors of both the intake and the transfer, and stows the intake.
     * @return the command
     */
    public static Command stopAllSubsystems(IntakeSubsystem intake, TransferSubsystem transfer, ShooterSubsystem shooter, TrapperSubsystem trapper) {
        return transfer.transferForceStop()
            .andThen(intake.stopRollers())
            .andThen(intake.moveIntakeUp())
            .andThen(shooter.stopShooter())
            .andThen(trapper.stopRollers());
    }

    public static Command climberUpCommand(ClimberSubsystem climber) {
        return climber.runClimber(0.5); // 0.75
    }

    public static Command climberDownCommand(ClimberSubsystem climber) {
        return climber.runClimber(-0.5);
    }
}
