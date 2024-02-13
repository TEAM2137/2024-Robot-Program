package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    private double currentThreshold = 60;
    
    private CANSparkMax pivotMotor;
    private CANSparkMax rollerMotor;

    private boolean forceStop;
    private boolean isRaised;
  
    public IntakeSubsystem() {
        super();

        pivotMotor = new CANSparkMax(CanIDs.get("intake-pivot"), CANSparkLowLevel.MotorType.kBrushless);
        rollerMotor = new CANSparkMax(CanIDs.get("intake-rollers"), CANSparkLowLevel.MotorType.kBrushless);
    }

    
    public Command startRollers() {
        return runOnce(() -> rollerMotor.set(-1));
    }

    public Command stopRollers() {
        return runOnce(() -> rollerMotor.set(0));
    }

    public Command moveIntakeDown(double speed) {
        return removeForceStop().andThen(
            runEnd(() -> {
                pivotMotor.set(-speed * 0.6);
            }, 
            () -> {
                pivotMotor.set(0);
            }
        ).until(() -> forceStop || pivotMotor.getOutputCurrent() >= currentThreshold / 2))
        .andThen(() -> isRaised = false);
    }

    public Command moveIntakeUp(double speed) {
        return removeForceStop().andThen(
            runEnd(() -> {
                pivotMotor.set(speed);
            }, 
            () -> {
                pivotMotor.set(0);
            }
        ).until(() -> forceStop || pivotMotor.getOutputCurrent() >= currentThreshold))
        .andThen(() -> isRaised = true);
    }

    public Command removeForceStop() {
        return runOnce(() -> forceStop = false);
    }

    public Command pivotForceStop() {
        return runOnce(() -> forceStop = true);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Intake Output Current", pivotMotor.getOutputCurrent());
        SmartDashboard.updateValues();
    }


    public Command togglePivot(double speed) {
        return runOnce(() -> {
            if (isRaised) {
                CommandScheduler.getInstance().schedule(moveIntakeDown(speed));
            } else CommandScheduler.getInstance().schedule(moveIntakeUp(speed));
        });
    }


    public Command deployIntake(double speed) {
        return startRollers().andThen(moveIntakeDown(speed));
    }
}
