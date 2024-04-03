package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.util.LEDs;

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
    private LEDs leds;

    // The threshold for input on the controller sticks (0.0 - 1.0)
    private double stickDeadzone = 0.2;

    // These ones are self explanatory
    private double driveSpeed = normalSpeed;
    private double rotationSpeed = 1.3;

    private double prevStickAngle = 0;
    private boolean isTargetingSpeaker = false;
    private boolean isTargetingHome = false;

    // Grabs values from the RobotContainer
    public Teleop(SwerveDrivetrain driveSubsystem, CommandXboxController driverController, CommandXboxController operatorController, LEDs leds) {
        this.driveSubsystem = driveSubsystem;
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.leds = leds;
    }

    public void init(ShooterSubsystem shooter, IntakeSubsystem intake, TransferSubsystem transfer, TrapperSubsystem trapper, ClimberSubsystem climber) {
        // +++ Init Subsystems +++

        intake.init();

        // +++ Start controller bindings +++

        // +++ DRIVER +++

        driverController.start().onTrue(Commands.runOnce(driveSubsystem::resetGyro)
            .andThen(RumbleSequences.rumbleOnce(driverController)));

        // driverController.back().onTrue(CommandSequences.rawShootCommand(0.8, transfer, shooter));
        driverController.b().onTrue(
            CommandSequences.transferToShooterCommand(intake, transfer, shooter, trapper)
                .andThen(shooter.stowPivot().andThen(cancelTargeting()).andThen(CommandSequences.stopAllSubsystems(intake, transfer, shooter, trapper))));

        // Aiming
        driverController.a().onTrue(startSpeakerAimCommand());
        driverController.rightBumper().onTrue(startHomeAimCommand());

        // Manual intake pivot
        driverController.back().onTrue(intake.togglePivot());

        // driverController.povUp().whileTrue(Commands.run(() -> shooter.changePivotTarget(0.3)));
        // driverController.povDown().whileTrue(Commands.run(() -> shooter.changePivotTarget(-0.3)));

        // Slow button
        driverController.leftTrigger().onTrue(new InstantCommand(() -> {
            driveSpeed = slowSpeed;
            rotationSpeed = 0.6;
        }));
        driverController.leftTrigger().onFalse(new InstantCommand(() -> {
            driveSpeed = normalSpeed;
            rotationSpeed = 1.3;
        }));

        // Intake phase
        driverController.rightTrigger().onTrue(CommandSequences.intakeAndTransfer(intake, transfer)
            .andThen(RumbleSequences.rumbleOnce(driverController)));
        // Force stop
        driverController.x().onTrue(cancelTargeting().andThen(
            CommandSequences.stopAllSubsystems(intake, transfer, shooter, trapper)));
        // X Lock
        driverController.y().whileTrue(Commands.run(() -> driveSubsystem.xLock()));

        // Stow command
        driverController.leftBumper().onTrue(cancelTargeting().andThen(
            CommandSequences.stopAllSubsystems(intake, transfer, shooter, trapper))
            .andThen(RumbleSequences.rumbleDualPulse(driverController).andThen(shooter.stowPivot())));

        // +++ OPERATOR +++

        operatorController.x().onTrue(intake.reverseRollers().andThen(transfer.reverse()));
        operatorController.x().onFalse(intake.stopRollers().andThen(transfer.transferForceStop()));

        // Shooter pivot manual controls
        // operatorController.rightBumper().onTrue(shooter.setPivotTarget(ShooterSubsystem.Constants.maxAngle)
        //     .andThen(CommandSequences.rawShootCommand(1, transfer, shooter)));
        operatorController.leftBumper().onTrue(shooter.setPivotTarget(ShooterSubsystem.Constants.manualClose)
            .andThen(CommandSequences.rawShootCommand(0.7, transfer, shooter)));

        operatorController.a().onTrue(trapper.runRollers(0.7));
        operatorController.a().onFalse(trapper.stopRollers());

        operatorController.rightTrigger().onTrue(trapper.homePosition());
        operatorController.leftTrigger().onTrue(trapper.ampPosition());

        operatorController.povUp().onTrue(trapper.climbPosition()
            .andThen(CommandSequences.climberUpCommand(climber)));
        operatorController.povUp().onFalse(climber.stopClimber());
        operatorController.povDown().onTrue(CommandSequences.climberDownCommand(climber));
        operatorController.povDown().onFalse(climber.stopClimber());

        // Shooter manual toggle
        operatorController.y().onTrue(CommandSequences.moveToShooterForArmCommand(trapper, shooter, transfer));
        operatorController.b().onTrue(CommandSequences.shootIntoArmCommand(trapper, shooter));

        // +++ End controller bindings +++

        // Schedule starting commands
        CommandScheduler.getInstance().schedule(shooter.stowPivot());
        CommandScheduler.getInstance().schedule(Commands.runOnce(() -> isTargetingSpeaker = false));
        CommandScheduler.getInstance().schedule(CommandSequences.stopAllSubsystems(intake, transfer, shooter, trapper));

        // Init teleop command
        driveSubsystem.resetModuleAngles();
        driveSubsystem.setDefaultCommand(getTeleopCommand(shooter));
    }

    // Setup the teleop drivetrain command
    public Command getTeleopCommand(ShooterSubsystem shooter) {
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
                if (isTargetingSpeaker) rot = targetUpdate(shooter, ShotLocation.SPEAKER);
                if (isTargetingHome) rot = targetUpdate(shooter, ShotLocation.HOME);

                driveSubsystem.driveTranslationRotationRaw(new ChassisSpeeds(speedY, speedX, rot));
            },
            driveSubsystem
        );
    }

    /**
     * @return the rotation error
     */
    public double targetUpdate(ShooterSubsystem shooter, ShotLocation location) {
        double rot = 0;
        Pair<Double, Double> data = getAimData(shooter, location);
        rot = data.getFirst();

        switch (location) {
            case SPEAKER:
                // Increasing offset makes the robot shoot lower
                shooter.setFromDistance(data.getSecond() + 0.33);
                break;
            // case HOME:
            default:
                shooter.setPowerRaw(0.4f);
                shooter.setPivotTargetRaw(42.5);
                break;
        }

        return rot;
    }

    public Command startSpeakerAimCommand() {
        return Commands.runOnce(() -> isTargetingSpeaker = true);
    }

    public Command startHomeAimCommand() {
        return Commands.runOnce(() -> isTargetingHome = true);
    }

    public Command cancelTargeting() {
        return Commands.runOnce(() -> { isTargetingSpeaker = false; isTargetingHome = false; });
    }

    public boolean isTargeting() { return isTargetingSpeaker || isTargetingHome; }
    public boolean isTargetingSpeaker() { return isTargetingSpeaker; }
    public boolean isTargetingHome() { return isTargetingHome; }

    public Pair<Double, Double> getAimData(ShooterSubsystem shooter, ShotLocation location) {
        // Flips the aiming if the alliance is blue
        boolean isBlueAlliance = true;
        isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

        // Sets where it should point (field space coords)
        Translation2d targetPos;
        switch (location) {
            case SPEAKER:
                if (isBlueAlliance) targetPos = new Translation2d(0.1, 5.53);
                else targetPos = new Translation2d(16.5, 5.53);
                break;
            // case HOME
            default:
                if (isBlueAlliance) targetPos = new Translation2d(3.0, 6);
                else targetPos = new Translation2d(13.6, 6);
                break;
        }
        driveSubsystem.targetPosePublisher.set(new Pose2d(targetPos, new Rotation2d(0)));

        // Get robot pose
        Pose2d robotPose = driveSubsystem.getPose();
        
        // Calculate stuff
        double distance = Math.hypot(targetPos.getX() - robotPose.getX(), targetPos.getY() - robotPose.getY());
        double desiredAngle = Math.atan2(targetPos.getY() - robotPose.getY(), targetPos.getX() - robotPose.getX());
        Rotation2d currentAngle = robotPose.getRotation();
        Rotation2d targetAngle = Rotation2d.fromRadians(desiredAngle + Math.PI);

        double kP = 0.035; // The amount of force it turns to the target with
        double error = -currentAngle.minus(targetAngle).getDegrees(); // Calculate error
        if (error > 20) error = 20; if (error < -20) error = -20;

        // Post debug values
        SmartDashboard.putNumber("Distance", distance);

        return new Pair<>(error * kP, distance);
    }

    public LEDs getLEDs() { return leds; }

    public enum ShotLocation {
        SPEAKER, HOME
    }
}
