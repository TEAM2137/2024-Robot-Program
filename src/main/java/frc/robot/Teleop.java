package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.vision.AprilTagVision;

public class Teleop {
    // Enables and disables field centric modes
    private final boolean fieldCentricMovement = true;
    private final boolean fieldCentricRotation = false;

    // Set speeds
    private final double slowSpeed = 0.5;
    private final double normalSpeed = 1.0;

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

    // Value of robot rotation before 180 degree command
    private Rotation2d prevRobotRot;

    // Grabs values from the RobotContainer
    public Teleop(SwerveDrivetrain driveSubsystem, CommandXboxController driverController,
            CommandXboxController operatorController, AprilTagVision vision) {
        this.driveSubsystem = driveSubsystem;
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.vision = vision;
    }

    public void init() {//ShooterSubsystem shooter, IntakeSubsystem intake, TransferSubsystem transfer, TrapperSubsystem trapper) {
        // // +++ Init Subsystems +++

        // intake.init();

        // // +++ Configure controller bindings here +++

        // Driver controller
        driverController.start().onTrue(Commands.runOnce(() -> driveSubsystem.resetGyro(), driveSubsystem));
        // driverController.b().onTrue(CommandSequences.speakerAimAndShootCommand(driveSubsystem, vision, transfer, shooter).andThen(RumbleSequences.rumbleDualPulse(driverController.getHID())));
        // // FOR DEMO: driverController.b().onTrue(CommandSequences.rawShootCommand(driveSubsystem, transfer, shooter));

        // Spin robot 180 degrees
        driverController.leftStick().onTrue(
            new InstantCommand(() -> prevRobotRot = driveSubsystem.getRobotAngle())
            .andThen(new RunCommand(() -> {
                Rotation2d target = prevRobotRot.plus(Rotation2d.fromDegrees(180));

                double error = target.minus(driveSubsystem.getRobotAngle()).getDegrees();
                driveSubsystem.driveTranslationRotationRaw(
                    new ChassisSpeeds(0, 0, error * 0.01 * rotationSpeed)
                );
        }, driveSubsystem)).until(() -> Math.abs(prevRobotRot.plus(Rotation2d.fromDegrees(180)).minus(driveSubsystem.getRobotAngle()).getDegrees()) < 1)); // Hoping this will turn the robot 180 degrees

        // // Slow buttons
        // driverController.leftTrigger().onTrue(new InstantCommand(() -> {
        //     driveSpeed = slowSpeed;
        //     rotationSpeed = 0.75;
        // }));
        // driverController.leftTrigger().onFalse(new InstantCommand(() -> {
        //     driveSpeed = normalSpeed;
        //     rotationSpeed = 1.3;
        // }));

        // // Shooting phase
        // operatorController.a().onTrue(CommandSequences.startShooterAndTransfer(0.75, shooter, transfer));
        // operatorController.a().onFalse(CommandSequences.stopShooterAndTransfer(shooter, transfer));

        // // Shooter warm-up
        // operatorController.y().onTrue(shooter.toggleShooter(0.5));

        // operatorController.b().onTrue(CommandSequences.moveToTrapper(trapper, shooter, transfer)
        //     .andThen(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)));

        // operatorController.rightTrigger().onTrue(trapper.moveToHomePosition());
        // operatorController.leftTrigger().onTrue(trapper.moveToFeedPosition());
        // //operatorController.y().onFalse(shooter.stopShooter());

        // // Intake phase
        // driverController.a().onTrue(CommandSequences.intakeAndTransfer(intake, transfer).andThen(RumbleSequences.rumbleOnce(driverController.getHID())));
        //     // Rumble controller to alert the driver of a note pickup
        //     // .andThen(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1))
        //     // .withTimeout(0.5).andThen(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));

        // driverController.x().onTrue(CommandSequences.stopAllSubsystems(intake, transfer, shooter).andThen(RumbleSequences.alternatingRumbles(driverController.getHID())));

        // driverController.rightBumper().onTrue(intake.togglePivot());
        // driverController.leftBumper().onTrue(shooter.stowPivot().andThen(RumbleSequences.rumbleDualPulse(driverController.getHID())));

        // operatorController.rightBumper().onTrue(shooter.setPivotTarget(ShooterSubsystem.Constants.longAngle));
        // operatorController.leftBumper().onTrue(shooter.setPivotTarget(ShooterSubsystem.Constants.shortAngle));

        // // Init teleop command
        // CommandScheduler.getInstance().schedule(intake.moveIntakeUp());
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
