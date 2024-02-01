package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.vision.AprilTagVision;

public class Teleop {
    // Enables and disables field centric modes
    private final boolean fieldCentricMovement = true;
    private final boolean fieldCentricRotation = false;

    // Declare controllers and necessary subsystems
    private CommandXboxController driverController;
    private CommandXboxController operatorController;
    private SwerveDrivetrain driveSubsystem;
    private AprilTagVision vision;

    // The threshold for input on the controller sticks (0.0 - 1.0)
    private double stickDeadzone = 0.3;

    // These ones are self explanatory
    private double driveSpeed = 0.6;
    private double rotationSpeed = 1.5;

    private double prevStickAngle = 0;

    // Grabs values from the RobotContainer
    public Teleop(SwerveDrivetrain driveSubsystem, CommandXboxController driverController,
            CommandXboxController operatorController, AprilTagVision vision) {
        this.driveSubsystem = driveSubsystem;
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.vision = vision;
    }

    public void init() {
        // Configure controller bindings here

        driverController.start().onTrue(Commands.runOnce(() -> driveSubsystem.resetGyro(), driveSubsystem));
        driverController.a().onTrue(pointToSpeakerCommand());

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

    // Uses the limelight and AprilTags to point towards the speaker target.
    public Command pointToSpeakerCommand() {
        return new RunCommand(
            () -> {
                // Where should it point (field space coords)
                double targetX = 8.308467;
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
}
