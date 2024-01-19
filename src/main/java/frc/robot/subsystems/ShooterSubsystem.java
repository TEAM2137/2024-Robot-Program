package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;
 
public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax launcherMotor1;
    private CANSparkMax launcherMotor2;

    private CANSparkMax pivotMotor;
    private RelativeEncoder pivotEncoder;
    private SparkPIDController pivotPID;

    public ShooterSubsystem() {
        super();
        launcherMotor1 = new CANSparkMax(CanIDs.get("launcher-1"), MotorType.kBrushless);
        launcherMotor2 = new CANSparkMax(CanIDs.get("launcher-2"), MotorType.kBrushless);

        pivotMotor = new CANSparkMax(CanIDs.get(""), MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPID = pivotMotor.getPIDController();
    }

    /**
     * @param time The amount of time that the launcher motors run for, in seconds
     * @param speed The speed that the motors move (0 - 1)
     * @return The command that runs the shooter
     */
    public Command runShooter(double time, double speed) {
        return run(() -> {
            launcherMotor1.set(speed);
            launcherMotor2.set(speed);
        }).withTimeout(time).andThen(runOnce(() -> {
            // Stop the motors when the time is up
            launcherMotor1.set(0);
            launcherMotor2.set(0);
        }));
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
