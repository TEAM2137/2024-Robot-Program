package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Teleop {
    private CommandXboxController driverController = new CommandXboxController(0);
    private CommandXboxController operatorController = new CommandXboxController(1);
    private SwerveDrivetrain driveSubsystem = new SwerveDrivetrain();

    private boolean moveForward;
    private boolean moveRight;

    public Teleop(SwerveDrivetrain driveSubsystem, CommandXboxController driverController,
            CommandXboxController operatorController) {
        this.driveSubsystem = driveSubsystem;
        this.driverController = driverController;
        this.operatorController = operatorController;
    }

    public void teleopInit() {
        // Configure controller bindings here

        // Bindings for swerve debug (delete these when necessary)
        driverController.a().onTrue(Commands.runOnce(() -> moveForward = true));
        driverController.a().onFalse(Commands.runOnce(() -> moveForward = false));
        driverController.b().onTrue(Commands.runOnce(() -> moveRight = true));
        driverController.b().onFalse(Commands.runOnce(() -> moveRight = false));
        
        driveSubsystem.setDefaultCommand(getTeleopCommand());
    }

    // Setup the actual teleop command
    public Command getTeleopCommand() {
        return new RunCommand(
            () -> {
                double direction = driveSubsystem.getRobotAngle().getRadians();

                double controllerX = -driverController.getLeftX();
                double controllerY = -driverController.getLeftY();

                // double rotationX = -driverController.getRightX();
                // double rotationY = -driverController.getRightY();
                // double rotation = Math.toDegrees(Math.atan2(rotationY, rotationX)) - 90;

                // rotation = (Math.abs(rotationX) < 0.3 && Math.abs(rotationY) < 0.3) ? 0 : rotation;
                // SmartDashboard.putNumber("Right Stick Rotation", rotation);

                double speedX = Math.sin(direction) * -controllerY + Math.cos(direction) * controllerX;
                double speedY = Math.cos(direction) * controllerY + Math.sin(direction) * controllerX;
                double rot = -driverController.getRightX();

                // Controller deadzones
                speedX = Math.abs(speedX) < 0.3 ? 0 : speedX;
                speedY = Math.abs(speedY) < 0.3 ? 0 : speedY;
                rot = Math.abs(rot) < 0.3 ? 0 : rot;
                
                driveSubsystem.driveTranslationRotationRaw(
                    new ChassisSpeeds(
                        moveForward ? 0.5 : speedY * 0.5,
                        moveRight ? 0.5 : speedX * 0.5,
                        rot * Math.PI
                    )
                );
            },
            driveSubsystem
        );
    }
}
