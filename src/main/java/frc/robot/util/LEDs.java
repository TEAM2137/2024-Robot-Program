package frc.robot.util;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    public CANifier canifier;

    public LEDs() {
        canifier = new CANifier(CanIDs.get("led-canifier"));
        onDisabled();
    }

    public void onDisabled() {
        setColor(LEDColor.GREEN);
    }

    public void setColor(LEDColor color) {
        canifier.setLEDOutput(color.getB(), LEDChannel.LEDChannelA);
        canifier.setLEDOutput(color.getR(), LEDChannel.LEDChannelB);
        canifier.setLEDOutput(color.getG(), LEDChannel.LEDChannelC);
    }

    public Command setColorCommand(LEDColor color) {
        return runOnce(() -> setColor(color));
    }

    public Command blinkLEDCommand(LEDColor colorA, LEDColor colorB, double interval) {
        return Commands.repeatingSequence(
            setColorCommand(colorA), Commands.waitSeconds(interval),
            setColorCommand(colorB), Commands.waitSeconds(interval)
        );
    }
}
