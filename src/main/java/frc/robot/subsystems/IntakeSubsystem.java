package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel;


 
public class IntakeSubsystem extends SubsystemBase {
    private final int pivotID = 40;
    private final int rollerID = 50;


    private CANSparkMax pivotMotor;
    private CANSparkMax rollerMotor;

  
    public IntakeSubsystem() {
        super();
        pivotMotor = new CANSparkMax(pivotID, CANSparkLowLevel.MotorType.kBrushless);
        rollerMotor = new CANSparkMax(rollerID, CANSparkLowLevel.MotorType.kBrushless);
    }

    
    public Command StartMotors() {
        return runOnce(() -> rollerMotor.set(1));
    }


}
