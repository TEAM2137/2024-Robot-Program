package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandSequences;
import frc.robot.commands.RumbleSequences;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TrapperSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.vision.VisionBlender;

public class Teleop {
    // Enables and disables field centric modes
    private final boolean fieldCentricMovement = true;
    private final boolean fieldCentricRotation = false;

    // Set speeds
    private final double slowSpeed = 0.4;
    private final double normalSpeed = 1.0;

    // Declare controllers and necessary subsystems
    private CommandXboxController driverController;
    private CommandXboxController operatorController;
    private SwerveDrivetrain driveSubsystem;
    private VisionBlender vision;

    // The threshold for input on the controller sticks (0.0 - 1.0)
    private double stickDeadzone = 0.2;

    // These ones are self explanatory
    private double driveSpeed = normalSpeed;
    private double rotationSpeed = 1.3;

    private double prevStickAngle = 0;

    // Grabs values from the RobotContainer
    public Teleop(SwerveDrivetrain driveSubsystem, CommandXboxController driverController,
            CommandXboxController operatorController, VisionBlender vision) {
        this.driveSubsystem = driveSubsystem;
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.vision = vision;
    }

    public void init(ShooterSubsystem shooter, IntakeSubsystem intake, TransferSubsystem transfer, TrapperSubsystem trapper, ClimberSubsystem climber) {
        // +++ Init Subsystems +++

        intake.init();

        // +++ Start controller bindings +++

        // +++ DRIVER +++

        driverController.start().onTrue(Commands.runOnce(() -> {
            driveSubsystem.resetGyro(); driveSubsystem.visionResetOdometry();
        }, driveSubsystem));
        driverController.b().onTrue(CommandSequences.speakerAimAndShootCommand(driveSubsystem, vision, transfer, shooter)
            .andThen(RumbleSequences.rumbleDualPulse(driverController.getHID())));

        // Slow button
        driverController.leftTrigger().onTrue(new InstantCommand(() -> {
            driveSpeed = slowSpeed;
            rotationSpeed = 0.75;
        }));
        driverController.leftTrigger().onFalse(new InstantCommand(() -> {
            driveSpeed = normalSpeed;
            rotationSpeed = 1.3;
        }));

        // Intake phase
        driverController.a().onTrue(CommandSequences.intakeAndTransfer(intake, transfer));//.andThen(RumbleSequences.rumbleOnce(driverController.getHID())));
        // Force stop
        driverController.x().onTrue(CommandSequences.stopAllSubsystems(intake, transfer, shooter));

        // Manual intake pivot
        driverController.rightBumper().onTrue(intake.togglePivot());
        // Stow command
        driverController.leftBumper().onTrue(shooter.stowPivot().andThen(RumbleSequences.rumbleDualPulse(driverController.getHID())));

        // +++ OPERATOR +++

        operatorController.x().onTrue(intake.reverseRollers().andThen(transfer.reverse()));
        operatorController.x().onFalse(intake.stopRollers().andThen(transfer.transferForceStop()));

        // Shooter pivot manual controls
        operatorController.rightBumper().onTrue(shooter.setPivotTarget(ShooterSubsystem.Constants.maxAngle)
            .andThen(CommandSequences.rawShootCommand(0.9, transfer, shooter)));
        operatorController.leftBumper().onTrue(shooter.setPivotTarget(ShooterSubsystem.Constants.midAngle)
            .andThen(CommandSequences.rawShootCommand(0.7, transfer, shooter)));

        operatorController.rightTrigger().onTrue(trapper.homePosition());
        operatorController.leftTrigger().onTrue(trapper.stage1());

        operatorController.povUp().onTrue(CommandSequences.climberUpCommand(climber));
        operatorController.povUp().onFalse(climber.stopClimber());
        operatorController.povDown().onTrue(CommandSequences.climberDownCommand(climber));
        operatorController.povDown().onFalse(climber.stopClimber());

        // Shooter manual toggle
        operatorController.y().onTrue(shooter.toggleShooter(0.5));
        operatorController.b().onTrue(shooter.setPivotTarget(ShooterSubsystem.Constants.ampAngle)
            .andThen(CommandSequences.ampShootCommand(0.2/* TODO tune this */, transfer, shooter)));

        // +++ End controller bindings +++

        // Init teleop command
        CommandScheduler.getInstance().schedule(intake.moveIntakeUp());
        driveSubsystem.setDefaultCommand(getTeleopCommand());
    }

    // Setup the teleop drivetrain command
    public Command getTeleopCommand() {
        return new RunCommand(
            () -> {
                // Controller + Pigeon inputs
                double direction = driveSubsystem.getRotation().getRadians();
                double controllerX = -driverController.getLeftX();
                double controllerY = -driverController.getLeftY();
                double rotationX = -driverController.getRightX();
                double rotationY = -driverController.getRightY();

                // Controller deadzones
                controllerX = Math.abs(controllerX) < stickDeadzone ? 0 : controllerX;
                controllerY = Math.abs(controllerY) < stickDeadzone ? 0 : controllerY;
                rotationX = Math.abs(rotationX) < stickDeadzone ? 0 : rotationX;
                rotationY = Math.abs(rotationY) < stickDeadzone ? 0 : rotationY;

                // Field-centric rotation calculations
                double stickAngle = prevStickAngle;
                if (Math.abs(rotationX) > stickDeadzone || Math.abs(rotationY) > stickDeadzone) {
                    stickAngle = -Math.toDegrees(Math.atan2(rotationY, rotationX)) + 90;
                    prevStickAngle = stickAngle;
                }

                double kP = 0.01;
                Rotation2d currentAngle = Rotation2d.fromRadians(direction); // Current angle
                Rotation2d targetAngle = Rotation2d.fromDegrees(stickAngle); // Desired angle
                double error = targetAngle.minus(currentAngle).getDegrees(); // Calculate error
                if (Math.abs(error * kP) < 0.005) error = 0;

                // Sets speeds and applies field centric if enabled
                double speedX = (fieldCentricMovement
                    ? Math.sin(direction) * -controllerY + 
                    Math.cos(direction) * controllerX /* Field centric */
                    : controllerX) /* Robot centric */ * driveSpeed;
                double speedY = (fieldCentricMovement 
                    ? Math.cos(direction) * controllerY + 
                    Math.sin(direction) * controllerX /* Field centric */
                    : controllerY) /* Robot centric */ * driveSpeed;
                double rot = (fieldCentricRotation
                    ? error * kP /* Field centric */
                    : rotationX) /* Robot centric */ * rotationSpeed;

                // Actually drive the swerve base
                driveSubsystem.driveTranslationRotationRaw(new ChassisSpeeds(speedY, speedX, rot));
            },
            driveSubsystem
        );
    }
}
