package frc.robot.subsystems;

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
        public static final double ClimbMax = 100;
        public static final double ClimbGearRatio = .4;

        public static PID climbPID = new PID(.1, 0.0, .01, .1);

        public static double HardStopThreshold = 70;
    }

    private CANSparkMax climbLeft;
    private CANSparkMax climbRight;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private SparkPIDController leftPID;
    private SparkPIDController rightPID;

    public ClimberSubsystem() {
        super();

        climbLeft = new CANSparkMax(CanIDs.get("climber-left"), CANSparkLowLevel.MotorType.kBrushless);
        climbRight = new CANSparkMax(CanIDs.get("climber-right"), CANSparkLowLevel.MotorType.kBrushless);

        leftEncoder = climbLeft.getEncoder();
        rightEncoder = climbRight.getEncoder();

        leftPID = climbRight.getPIDController();
        leftPID.setFeedbackDevice(leftEncoder);
        leftPID.setP(Constants.climbPID.getP());
        leftPID.setI(Constants.climbPID.getI());
        leftPID.setD(Constants.climbPID.getD());
        leftPID.setFF(Constants.climbPID.getFF());

        rightPID = climbRight.getPIDController();
        rightPID.setFeedbackDevice(rightEncoder);
        rightPID.setP(Constants.climbPID.getP());
        rightPID.setI(Constants.climbPID.getI());
        rightPID.setD(Constants.climbPID.getD());
        rightPID.setFF(Constants.climbPID.getFF());
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

    /**
     * Command to move the climber all the way up
     * @return the command
     */
    public Command climberUpCommand() {
        return 
            setSpeedCommand(0.5)
            .alongWith(run(() -> {}))
            .until(() -> climbRight.getOutputCurrent() > Constants.HardStopThreshold)
            .andThen(stopClimber());
    }

    /**
     * Command to move the climber all the way down
     * @return the command
     */
    public Command climberDownCommand() {
        return 
            setSpeedCommand(-0.5)
            .alongWith(run(() -> {}))
            .until(() -> climbRight.getOutputCurrent() > Constants.HardStopThreshold)
            .andThen(stopClimber());
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Climb Position Raw", rightEncoder.getPosition());
        SmartDashboard.putNumber("Climb Position %", rightEncoder.getPosition() / Constants.ClimbMax * 100.0);
        SmartDashboard.updateValues();
    }
}
