package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.vision.AprilTagVision;

public class Teleop {
    // Enables and disables field centric modes
    private final boolean fieldCentricMovement = true;
    private final boolean fieldCentricRotation = false;

    // Set speeds
    private final double slowSpeed = 0.5;
    private final double normalSpeed = 0.6;
    private final double fullSpeed = 0.9;

    // Declare controllers and necessary subsystems
    private CommandXboxController driverController;
    private CommandXboxController operatorController;
    private SwerveDrivetrain driveSubsystem;
    private AprilTagVision vision;

    // The threshold for input on the controller sticks (0.0 - 1.0)
    private double stickDeadzone = 0.3;

    // These ones are self explanatory
    private double driveSpeed = normalSpeed;
    private double rotationSpeed = 1.3;

    private double prevStickAngle = 0;

    // Grabs values from the RobotContainer
    public Teleop(SwerveDrivetrain driveSubsystem, CommandXboxController driverController,
            CommandXboxController operatorController, AprilTagVision vision) {
        this.driveSubsystem = driveSubsystem;
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.vision = vision;
    }

    public void init(ShooterSubsystem shooter, IntakeSubsystem intake, TransferSubsystem transfer) {
        // +++ Configure controller bindings here +++

        // Driver controller
        driverController.start().onTrue(Commands.runOnce(() -> driveSubsystem.resetGyro(), driveSubsystem));
        driverController.b().onTrue(CommandSequences.speakerAimAndShootCommand(driveSubsystem, vision, transfer, shooter));
        
        // Slow buttons
        driverController.leftTrigger().onTrue(new InstantCommand(() -> {
            driveSpeed = slowSpeed;
            rotationSpeed = 1;
        }));
        driverController.leftTrigger().onFalse(new InstantCommand(() -> {
            driveSpeed = normalSpeed;
            rotationSpeed = 1.3;
        }));
        driverController.rightTrigger().onTrue(new InstantCommand(() -> {
            driveSpeed = fullSpeed;
            rotationSpeed = 1.6;
        }));
        driverController.rightTrigger().onFalse(new InstantCommand(() ->  {
            driveSpeed = normalSpeed;
            rotationSpeed = 1.3;
        }));

        // Shooting phase
        operatorController.a().onTrue(CommandSequences.startShooterAndTransfer(0.75, shooter, transfer));
        operatorController.a().onFalse(CommandSequences.stopShooterAndTransfer(shooter, transfer));

        // Shooter warm-up
        operatorController.y().onTrue(shooter.toggleShooter(0.75));
        operatorController.x().onTrue(intake.startRollers());
        //operatorController.y().onFalse(shooter.stopShooter());

        // Intake phase
        driverController.a().onTrue(CommandSequences.intakeAndTransfer(intake, transfer));
        driverController.x().onTrue(CommandSequences.stopIntakeAndTransfer(intake, transfer));

        driverController.rightBumper().onTrue(intake.togglePivot(0.4));
        driverController.leftBumper().onTrue(shooter.stowPivot());

        operatorController.rightBumper().onTrue(shooter.setPivotTarget(ShooterSubsystem.Constants.longAngle));
        operatorController.leftBumper().onTrue(shooter.setPivotTarget(ShooterSubsystem.Constants.shortAngle));

        // Init teleop command
        CommandScheduler.getInstance().schedule(intake.moveIntakeUp(0.4));
        // Don't reset on start: driveSubsystem.resetGyro();
        driveSubsystem.setDefaultCommand(getTeleopCommand());
    }

    // Setup the teleop drivetrain command
    public Command getTeleopCommand() {
        return new RunCommand(
            () -> {
                // Controller + Pigeon inputs
                double direction = driveSubsystem.getRobotAngle().getRadians();
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
                driveSubsystem.driveTranslationRotationRaw(
                    new ChassisSpeeds(speedY, speedX, rot)
                );
            },
            driveSubsystem
        );
    }
}
