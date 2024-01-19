package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel;


 
public class IntakeSubsystem extends SubsystemBase {
    private final int pivotID = 40;
    private final int rollerID = 50;


    private CANSparkMax pivotMotor;
    private CANSparkMax rollerMotor;

    private SparkPIDController pivotPIDController;
    private SparkPIDController rollerPIDController;

    private RelativeEncoder pivotEncoder;


  
    public IntakeSubsystem() {
        super();

        pivotMotor = new CANSparkMax(pivotID, CANSparkLowLevel.MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPIDController = pivotMotor.getPIDController();
        pivotPIDController.setFeedbackDevice(pivotEncoder);
        pivotPIDController.setP(0.1);
        pivotPIDController.setI(0.2);
        pivotPIDController.setD(0.3);
        pivotPIDController.setFF(0.4);

        rollerMotor = new CANSparkMax(rollerID, CANSparkLowLevel.MotorType.kBrushless);
        rollerPIDController = rollerMotor.getPIDController();
        rollerPIDController.setP(0.01);
        rollerPIDController.setI(0.02);
        rollerPIDController.setD(0.03);
        rollerPIDController.setFF(0.04);
    }

    
    public Command StartMotors() {
        return runOnce(() -> rollerMotor.set(1));
    }


}
