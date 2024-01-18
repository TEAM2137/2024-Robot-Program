package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax launcherMotor1;
    private CANSparkMax launcherMotor2;
    //private CANSparkMax pivotMotor;

    public ShooterSubsystem() {
        super();
        launcherMotor1 = new CANSparkMax(0, MotorType.kBrushless);
        launcherMotor2 = new CANSparkMax(0, MotorType.kBrushless);
        //pivotMotor = new CANSparkMax(0, MotorType.kBrushless);
    }

    /**
     * @param time The amount of time that the launcher motors run for, in seconds
     * @param speed The speed that the motors move (0 - 1)
     * @return The command that runs the shooter
     */
    public Command runShooter(double time, double speed) {
        return new RunCommand(
            () -> {
                launcherMotor1.set(speed);
                launcherMotor2.set(speed);
            }
        ).withTimeout(time).andThen(runOnce(() -> {
            launcherMotor1.set(0);
            launcherMotor2.set(0);
        }));
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
