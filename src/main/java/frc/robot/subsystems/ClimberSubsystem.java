package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CanIDs;
import frc.robot.util.PID;

public class ClimberSubsystem extends SubsystemBase {
    public static class Constants {
        public static final double ClimbMax = 100; // Motor rotations at which climb maxes out

        public static final double ClimbGearRatio = .4;

        public static PID climbPID = new PID(.1, 0.0, .01, .1);
    }

    private CANSparkMax climbMotor;
    private RelativeEncoder climbEncoder;

    private SparkPIDController climbPID;

    public ClimberSubsystem() {
        super();

        climbMotor = new CANSparkMax(CanIDs.get("climber"), CANSparkLowLevel.MotorType.kBrushless);
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

    /**
     * Command to set the climber position
     * @param percentage percent of the max height to move the climber to, from 0.0 to 1.0
     * @return Command that moves the climber
     */
    public Command setClimberPositionCommand(double percentage) {
        return runOnce(() -> climbPID.setReference((percentage * Constants.ClimbMax), CANSparkBase.ControlType.kPosition));
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Climb Position Raw", climbEncoder.getPosition());
        SmartDashboard.putNumber("Climb Position %", climbEncoder.getPosition() / Constants.ClimbMax);
        SmartDashboard.updateValues();
    }
}
