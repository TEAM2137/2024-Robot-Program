package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private double driveSpeed = 0.5;
    private double rotationSpeed = Math.PI;

    // Grabs values from the RobotContainer
    public Teleop(SwerveDrivetrain driveSubsystem, CommandXboxController driverController,
            CommandXboxController operatorController) {
        this.driveSubsystem = driveSubsystem;
        this.driverController = driverController;
        this.operatorController = operatorController;
    }

    public void init() {
        // Configure controller bindings here
        // TODO: Bind commands to controllers

        // Init teleop command
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
                double stickAngle = Math.toDegrees(Math.atan2(rotationY, rotationX)) - 90;
                stickAngle = (Math.abs(rotationX) < 0.3 && Math.abs(rotationY) < 0.3) ? 0 : stickAngle;
                SmartDashboard.putNumber("Right Stick Angle", stickAngle);

                double kP = 0.005;
                Rotation2d currentAngle = Rotation2d.fromDegrees(direction); // Current angle
                Rotation2d targetAngle = Rotation2d.fromDegrees(stickAngle); // Desired angle
                double error = targetAngle.minus(currentAngle).getDegrees(); // Calculate error

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
                    new ChassisSpeeds(speedY * 0.5, speedX * 0.5, rot)
                );
            },
            driveSubsystem
        );
    }
}
