package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * This is an example class so you all have a reference for Command factories.
 * This represents a subsystem that keeps track of a boolean and a counter.
 * Comments over each function explains what they do.
 * - Gage
 */

 
 // Make sure to extend SubsystemBase
public class PretendSubsystem extends SubsystemBase {
    private int fakeCounter = 0;
    private boolean fakeToggle = false;

    // Initialize things that need it here. Remember to call super() though so the subsystem gets registered.
    public PretendSubsystem() {
        super();
    }

    // This increases the counter once. runOnce runs whatever expression is inside the parentheses. If you need more lines, you could use brackets.
    public Command countUpCommand() {
        return this.runOnce(() -> fakeCounter++);
    }

    // This counts upwards until the command is interrupted (it ends). You could add a .withTimeout() to the end of this to force it to quit after some time.
    public Command countUntilInterruptCommand() {
        return this.run(() -> fakeCounter++);
    }

    // Here's an example of a command with a parameter.
    public Command setCounterCommand(int value) {
        return this.runOnce(() -> fakeCounter = value);
    }

    // This counts upwards once and then when the command ends it flips the boolean. The first parameter is the code run at the start, and the second is the ending code.
    public Command countAndFlipCommand() {
        return this.startEnd(() -> fakeCounter++, () -> fakeToggle = !fakeToggle);
    }

    // This counts upwards until it interrupts, but when it interrupts it flips the boolean.
    public Command countContinuousAndFlipCommand() {
        return this.runEnd(() -> fakeCounter++, () -> fakeToggle = !fakeToggle);
    }

    // Update function for the subsystem. Anything that needs to run continuously, like SmartDashboard updates, goes here.
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Example Counter", fakeCounter);
        SmartDashboard.putBoolean("Example Bool", fakeToggle);
        SmartDashboard.updateValues();
        super.periodic();
    }
}
