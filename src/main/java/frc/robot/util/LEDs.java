package frc.robot.util;

import java.util.function.Supplier;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    public CANifier canifier;
    public float brightness = 0.5f;

    public LEDs() {
        canifier = new CANifier(CanIDs.get("led-canifier"));
        setDefaultCommand(setColorCommand(LEDColor.BLUE));
        onDisabled();
    }

    /**
     * Called when the OpMode is disabled
     */
    public void onDisabled() {
        if (!DriverStation.isFMSAttached()) setColor(LEDColor.NONE.mul(0.25f));
    }

    /**
     * Sets the color of the LEDs
     * @param color The color to display
     */
    public void setColor(LEDColor color) {
        canifier.setLEDOutput(color.getG() * brightness, LEDChannel.LEDChannelA);
        canifier.setLEDOutput(color.getR() * brightness, LEDChannel.LEDChannelB);
        canifier.setLEDOutput(color.getB() * brightness, LEDChannel.LEDChannelC);
    }

    /**
     * A command that sets the color of the LEDs
     * @param color The color to display
     * @return The command
     */
    public Command setColorCommand(LEDColor color) {
        return runOnce(() -> setColor(color));
    }

    /**
     * Same as {@code setColorCommand()} except it isn't instant
     * @param color The color to display
     * @return The command
     */
    public Command holdColorCommand(LEDColor color) {
        return run(() -> setColor(color));
    }

    /**
     * A command that blinks between two colors
     * @param colorA The first color to blink
     * @param colorB The second color to blink
     * @param interval The time between blinks
     * @param times The amount of times it blinks
     * @return The command
     */
    public Command blinkColorCommand(LEDColor colorA, LEDColor colorB, double interval, int times) {
        return blinkColorCommand(colorA, colorB, interval).withTimeout(interval * 2 * times);
    }

    /**
     * A command that blinks between two colors
     * @param colorA The first color to blink
     * @param colorB The second color to blink
     * @param interval The time between blinks
     * @param times The amount of times it blinks
     * @param onA A command that will be run when color A is shown
     * @param onB A command that will be run when color B is shown
     * @return The command
     */
    public Command blinkColorCommand(LEDColor colorA, LEDColor colorB, double interval, int times, Supplier<Command> onA, Supplier<Command> onB) {
        return blinkColorCommand(colorA, colorB, interval, onA, onB).withTimeout(interval * 2 * times);
    }

    /**
     * A command that blinks between two colors
     * @param colorA The first color to blink
     * @param colorB The second color to blink
     * @param interval The time between blinks
     * @return The command
     */
    public Command blinkColorCommand(LEDColor colorA, LEDColor colorB, double interval) {
        return Commands.repeatingSequence(
            setColorCommand(colorA), Commands.waitSeconds(interval),
            setColorCommand(colorB), Commands.waitSeconds(interval)
        );
    }

    /**
     * A command that blinks between two colors
     * @param colorA The first color to blink
     * @param colorB The second color to blink
     * @param interval The time between blinks
     * @param onA A command that will be run when color A is shown
     * @param onB A command that will be run when color B is shown
     * @return The command
     */
    public Command blinkColorCommand(LEDColor colorA, LEDColor colorB, double interval, Supplier<Command> onA, Supplier<Command> onB) {
        return Commands.repeatingSequence(
            setColorCommand(colorA).alongWith(onA.get()), Commands.waitSeconds(interval),
            setColorCommand(colorB).alongWith(onB.get()), Commands.waitSeconds(interval)
        );
    }
}
