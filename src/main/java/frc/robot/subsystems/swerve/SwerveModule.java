package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public abstract class SwerveModule extends SubsystemBase {
    public final String moduleName;
    public final String rioCanBus;
    public final String drivetrainCanBus;
    public double encoderOffset;
    public double currentPosition;

    public SwerveModule(int driveID, int turningID, int encoderID, double encoderOffset, String moduleName) {
        this.moduleName = moduleName;
        this.encoderOffset = encoderOffset;
        this.drivetrainCanBus = RobotContainer.getDrivetrainCanBusName();
        this.rioCanBus = RobotContainer.getRioCanBusName();
    }

    public SwerveModule(SwerveDrivetrain.Constants.SwerveModuleConstants constants) {
        this(constants.driveID, constants.turningID, constants.encoderID, constants.offset, constants.moduleName);
    }

    // @Override
    // public void periodic() {
    //     super.periodic();
    // }

    public abstract Rotation2d getModuleRotation();
    public abstract void setTurningTarget(Rotation2d target);
    public abstract void homeTurningMotor();

    public abstract void setDrivePowerRaw(double power);
    public abstract void setDriveVelocity(double velocity);
    public abstract double getDriveVelocity();
    public abstract double getDriveDistance();
    public abstract void resetDriveEncoder();

    public abstract SwerveModuleState getSwerveModuleState();
    public abstract void selfTargetAngle();

    public abstract void setDriveMode(boolean brake);

    public abstract void setTurnBrakeMode(boolean brake);
}
