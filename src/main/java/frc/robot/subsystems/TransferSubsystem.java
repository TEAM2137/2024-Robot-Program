package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.util.CanIDs;
import frc.robot.util.LEDColor;
import frc.robot.util.LEDs;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase {
    private boolean motorsStopped = false;

    private CANSparkMax beltMotor;

    // inBeamBreak is mounted so it's broken when a NOTE is fully in the transfer
    private DigitalInput inBeamBreak = new DigitalInput(0);
    private DigitalInput secondBeamBreak = new DigitalInput(1);

    public TransferSubsystem() {
        super();

        beltMotor = new CANSparkMax(CanIDs.get("transfer-motor"), CANSparkLowLevel.MotorType.kBrushless);
        beltMotor.setIdleMode(IdleMode.kCoast);
    }

    /**
     * Command to intake a NOTE into the transfer, which runs until a stop condition is met or a note is in the transfer
     * @param earlyStop Condition to stop early
     * @return The command
     */
    public Command intakeNoteCommand() {
        return removeForceStop().andThen(runEnd(
            () -> {
                beltMotor.set(0.6);
            },
            () -> {
                beltMotor.set(0);
            }
        ).until(() -> !inBeamBreak.get() || motorsStopped).andThen(() -> {
            motorsStopped = false;
        })); // Stop when the beam breaks
    }

    public Command reverse() {
        return removeForceStop().andThen(runEnd(
            () -> beltMotor.set(-0.5),
            () -> {
                beltMotor.set(0);
            }
        ).until(() ->  motorsStopped)); // Stop when beam breaks
    }

    /**
     * Re-enables the transfer motors. This should be run before most commands.
     * @return the command
     */
    public Command removeForceStop() {
        return runOnce(() -> motorsStopped = false);
    }

    /**
     * Shuts off the transfer belt motors
     * @return the command
     */
    public Command transferForceStop() {
        return runOnce(() -> motorsStopped = true);
    }

    /**
     * Command to feed the NOTE into the trapper
     * @return the command
     */
    public Command feedTrapperCommand() {
        return removeForceStop().andThen(runEnd(
            () -> beltMotor.set(0.3),
            () -> {
                beltMotor.set(0);
            }
        ).until(() -> !inBeamBreak.get() || motorsStopped)); // Stop when beam breaks
    }

    /**
     * Command to feed the NOTE into the shooter
     * @return The command
     */
    public Command feedShooterCommand() {
        return removeForceStop().andThen(runEnd(
            () -> {
                beltMotor.set(1.0);
            },
            () -> {
                beltMotor.set(0);
            }
        ).until(() -> motorsStopped));
    }

    public Command feedArmCommand() {
        return removeForceStop().andThen(runEnd(
            () -> {
                beltMotor.set(0.6);
            },
            () -> {
                beltMotor.set(0);
            }
        ).until(() -> motorsStopped || !secondBeamBreak.get()))
        .andThen(runEnd(
            () -> {
                beltMotor.set(0.6);
            },
            () -> {
                beltMotor.set(0);
            }
        ).until(() -> motorsStopped || secondBeamBreak.get()));
    }

    public boolean getOccupied() {
        return inBeamBreak.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!inBeamBreak.get()) {
            LEDs leds = RobotContainer.getInstance().teleop.getLEDs();
            
            CommandScheduler.getInstance().schedule(leds.holdColorCommand(LEDColor.YELLOW)
                .until(() -> inBeamBreak.get()));
        }

        SmartDashboard.putNumber("Transfer Power", beltMotor.get());
        SmartDashboard.putBoolean("Has Ring", !inBeamBreak.get());
        SmartDashboard.updateValues();
    }
}
