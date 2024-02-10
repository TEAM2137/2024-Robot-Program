package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel;

import frc.robot.util.CanIDs;
import frc.robot.util.PID;

// everything number is a placeholder
public class IntakeSubsystem extends SubsystemBase {

    public static class Constants {
        public static PID rollerPID = new PID(0.1, 0.2, 0.3, 0.4);
        public static PID pivotPID = new PID(0.01, 0.02, 0.03, 0.04);
    }
    
    private CANSparkMax pivotMotor;
    private CANSparkMax rollerMotor;

    private SparkPIDController pivotPIDController;
    private SparkPIDController rollerPIDController;

    private RelativeEncoder pivotEncoder;
  
    public IntakeSubsystem() {
        super();

        pivotMotor = new CANSparkMax(CanIDs.get("intake-pivot"), CANSparkLowLevel.MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();

        pivotPIDController = pivotMotor.getPIDController();
        pivotPIDController.setFeedbackDevice(pivotEncoder);
        pivotPIDController.setP(Constants.pivotPID.getP());
        pivotPIDController.setI(Constants.pivotPID.getI());
        pivotPIDController.setD(Constants.pivotPID.getD());
        pivotPIDController.setFF(Constants.pivotPID.getFF());

        rollerMotor = new CANSparkMax(CanIDs.get("intake-rollers"), CANSparkLowLevel.MotorType.kBrushless);

        rollerPIDController = rollerMotor.getPIDController();
        rollerPIDController.setP(Constants.rollerPID.getP());
        rollerPIDController.setI(Constants.rollerPID.getI());
        rollerPIDController.setD(Constants.rollerPID.getD());
        rollerPIDController.setFF(Constants.rollerPID.getFF());
    }

    
    public Command startMotors() {
        return runOnce(() -> rollerMotor.set(1));
    }

    public Command pleaseStop() {
        return runOnce(() -> rollerMotor.set(0));
    }

    public Command moveIntakeDown() {
        return runOnce(() -> pivotMotor.set(0.3)).alongWith(run(() -> {})).until(() -> pivotMotor.getOutputCurrent() > 1).andThen(() -> pivotMotor.set(0));
    }

    public Command moveIntakeUp() {
        return runOnce(() -> pivotMotor.set(-0.3)).alongWith(run(() -> {})).until(() -> pivotMotor.getOutputCurrent() > 1).andThen(() -> pivotMotor.set(0));
    }
}
