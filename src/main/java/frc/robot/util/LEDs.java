package frc.robot.util;

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

    public void onDisabled() {
        if (!DriverStation.isFMSAttached()) setColor(LEDColor.GREEN.mul(0.25f));
    }

    public void setColor(LEDColor color) {
        canifier.setLEDOutput(color.getG() * brightness, LEDChannel.LEDChannelA);
        canifier.setLEDOutput(color.getR() * brightness, LEDChannel.LEDChannelB);
        canifier.setLEDOutput(color.getB() * brightness, LEDChannel.LEDChannelC);
    }

    public Command setColorCommand(LEDColor color) {
        return runOnce(() -> setColor(color));
    }

    public Command holdColorCommand(LEDColor color) {
        return run(() -> setColor(color));
    }

    public Command blinkColorCommand(LEDColor colorA, LEDColor colorB, double interval, int times) {
        return blinkColorCommand(colorA, colorB, interval).withTimeout(interval * 2 * times);
    }

    public Command blinkColorCommand(LEDColor colorA, LEDColor colorB, double interval) {
        return Commands.repeatingSequence(
            setColorCommand(colorA), Commands.waitSeconds(interval),
            setColorCommand(colorB), Commands.waitSeconds(interval)
        );
    }
}
