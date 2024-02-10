package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Teleop {
    // Enables and disables field centric modes
    private final boolean fieldCentricMovement = true;
    private final boolean fieldCentricRotation = false;

    // Declare controllers and necessary subsystems
    private CommandXboxController driverController;
    private CommandXboxController operatorController;
    private SwerveDrivetrain driveSubsystem;

    // The threshold for input on the controller sticks (0.0 - 1.0)
    private double stickDeadzone = 0.3;

    // These ones are self explanatory
    private double driveSpeed = 0.6;
    private double rotationSpeed = 1.5;

    private double prevStickAngle = 0;

    // Grabs values from the RobotContainer
    public Teleop(SwerveDrivetrain driveSubsystem, CommandXboxController driverController,
            CommandXboxController operatorController) {
        this.driveSubsystem = driveSubsystem;
        this.driverController = driverController;
        this.operatorController = operatorController;
    }

    public void init(ShooterSubsystem shooter, IntakeSubsystem intake, TransferSubsystem transfer) {
        // Configure controller bindings here

        driverController.start().onTrue(Commands.runOnce(() -> driveSubsystem.resetGyro(), driveSubsystem));

        // Shooter
        operatorController.a().onTrue(shooter.startShooter(1));
        operatorController.a().onFalse(shooter.stopShooter());

        // Transfer
        operatorController.x().onTrue(transfer.intakeCommand(() -> true));
        operatorController.x().onFalse(transfer.forceStopTransfer());

        // Intake
        operatorController.b().onTrue(intake.startMotors());
        operatorController.b().onFalse(intake.stopIntake());
        operatorController.rightBumper().onTrue(intake.moveIntakeDown(0.3));
        operatorController.rightBumper().onFalse(intake.pivotForceStop());
        operatorController.leftBumper().onTrue(intake.moveIntakeDown(0.3));
        operatorController.leftBumper().onFalse(intake.pivotForceStop());

        // Init teleop command
        driveSubsystem.resetGyro();
        driveSubsystem.setDefaultCommand(getTeleopCommand());
    }

    // Setup the actual teleop command
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
