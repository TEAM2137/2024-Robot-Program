package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax climbLeft;
    private CANSparkMax climbRight;

    private boolean forceStop;

    public ClimberSubsystem() {
        super();

        climbLeft = new CANSparkMax(CanIDs.get("climber-left"), CANSparkLowLevel.MotorType.kBrushless);
        climbRight = new CANSparkMax(CanIDs.get("climber-right"), CANSparkLowLevel.MotorType.kBrushless);

        // setDefaultCommand(run(() -> {
        //     climbLeft.set(-0.015);
        //     climbRight.set(-0.015);
        // }));
    }

    /**
     * Command to set the climb motor speed.
     * @param speed Speed from -1.0 to 1.0
     * @return Command that sets the speed
     */
    public Command runClimber(double speed) {
        return removeForceStop().andThen(runEnd(() -> {
            climbLeft.set(speed);
            climbRight.set(speed);
        }, () -> {
            climbLeft.stopMotor();
            climbRight.stopMotor();
        }).until(() -> forceStop));
    }

    /**
     * Command to immediately stop the climb motor
     * @return Command that stops the motor
     */
    public Command stopClimber() {
        return runOnce(() -> forceStop = true);
    }

    public Command removeForceStop() {
        return runOnce(() -> forceStop = false);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
