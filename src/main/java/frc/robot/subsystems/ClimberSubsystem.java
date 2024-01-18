package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    public static class Constants {
        public static final int ClimbID = 70;

        public static final double ClimbMax = 100;
    }

    private CANSparkMax climbMotor;
    private RelativeEncoder climbEncoder;

    private SparkPIDController climbPID;

    public ClimberSubsystem() {
        super();

        climbMotor = new CANSparkMax(Constants.ClimbID, CANSparkLowLevel.MotorType.kBrushless);
        climbEncoder = climbMotor.getEncoder();

        climbPID = climbMotor.getPIDController();
    }

    /**
     * Command to set the climb motor speed.
     * @param speed Speed from -1.0 to 1.0
     * @return Command that sets the speed
     */
    public Command setSpeedCommand(double speed) {
        return runOnce(() -> climbMotor.set(speed));
    }

    /**
     * Command to immediately stop the climb motor
     * @return Command that stops the motor
     */
    public Command youShallNotPass() {
        return runOnce(() -> climbMotor.stopMotor());
    }
}
