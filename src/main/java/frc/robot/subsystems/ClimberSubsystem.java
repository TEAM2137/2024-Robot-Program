package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;

public class ClimberSubsystem extends SubsystemBase {
    public static class Constants {

    }

    private CANSparkMax climbLeft;
    private CANSparkMax climbRight;

    public ClimberSubsystem() {
        super();

        climbLeft = new CANSparkMax(CanIDs.get("climber-left"), CANSparkLowLevel.MotorType.kBrushless);
        climbRight = new CANSparkMax(CanIDs.get("climber-right"), CANSparkLowLevel.MotorType.kBrushless);
    }

    /**
     * Command to set the climb motor speed.
     * @param speed Speed from -1.0 to 1.0
     * @return Command that sets the speed
     */
    public Command setSpeedCommand(double speed) {
        return runOnce(() -> {
            climbLeft.set(speed);
            climbRight.set(speed);
        });
    }

    /**
     * Command to immediately stop the climb motor
     * @return Command that stops the motor
     */
    public Command stopClimber() {
        return runOnce(() -> {
            climbLeft.stopMotor();
            climbRight.stopMotor();
        });
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
