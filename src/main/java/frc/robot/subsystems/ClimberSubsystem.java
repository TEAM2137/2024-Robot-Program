package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PID;

public class ClimberSubsystem extends SubsystemBase {
    public static class Constants {
        // All placeholder values to be changed later
        public static final int ClimbID = 70;

        public static final double ClimbMax = 100; // Motor rotations at which climb maxes out

        public static final double ClimbGearRatio = .4;

        public static PID climbPID = new PID(.1, 0.0, .01, .1);
    }

    private CANSparkMax climbMotor;
    private RelativeEncoder climbEncoder;

    private SparkPIDController climbPID;

    public ClimberSubsystem() {
        super();

        climbMotor = new CANSparkMax(Constants.ClimbID, CANSparkLowLevel.MotorType.kBrushless);
        climbEncoder = climbMotor.getEncoder();

        climbPID = climbMotor.getPIDController();
        climbPID.setFeedbackDevice(climbEncoder);
        climbPID.setP(Constants.climbPID.getP());
        climbPID.setI(Constants.climbPID.getI());
        climbPID.setD(Constants.climbPID.getD());
        climbPID.setFF(Constants.climbPID.getFF());
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

    public Command setClimberPositionCommand(double percentage) {
        return runOnce(() -> climbPID.setReference((percentage * Constants.ClimbMax), CANSparkBase.ControlType.kPosition));
    }
}
