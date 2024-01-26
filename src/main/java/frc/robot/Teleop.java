package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public void teleopInit() {
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

                // Field-centric rotation calculations, these are temporary I think
                double rotation = Math.toDegrees(Math.atan2(rotationY, rotationX)) - 90;
                rotation = (Math.abs(rotationX) < 0.3 && Math.abs(rotationY) < 0.3) ? 0 : rotation;
                SmartDashboard.putNumber("Right Stick Rotation", rotation);

                // Sets speeds and applies field centric if enabled
                double speedX = (fieldCentricMovement
                    ? Math.sin(direction) * -controllerY + Math.cos(direction) * controllerX
                    : controllerX) * driveSpeed;
                double speedY = (fieldCentricMovement 
                    ? Math.cos(direction) * controllerY + Math.sin(direction) * controllerX
                    : controllerY) * driveSpeed;
                double rot = (fieldCentricRotation 
                    ? 0 // TODO: field centric rotation
                    : rotationX) * rotationSpeed;
                
                // Actually drive the swerve base
                driveSubsystem.driveTranslationRotationRaw(
                    new ChassisSpeeds(speedY * 0.5, speedX * 0.5, rot)
                );
            },
            driveSubsystem
        );
    }
}
