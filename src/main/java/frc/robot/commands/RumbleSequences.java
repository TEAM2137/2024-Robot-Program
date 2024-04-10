package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleSequences {
    public static Command rumbleDualPulse(CommandXboxController controller) {
        return rumble(controller, RumbleType.kBothRumble, 1).withTimeout(0.1)
            .andThen(shutOffRumble(controller)).andThen(Commands.waitSeconds(0.1))
            .andThen(rumble(controller, RumbleType.kBothRumble, 1)).withTimeout(0.1)
            .andThen(shutOffRumble(controller));
    }

    public static Command rumbleOnce(CommandXboxController driverController) {
        return rumble(driverController, RumbleType.kBothRumble, 1).withTimeout(0.2)
            .andThen(shutOffRumble(driverController));
    }

    public static Command rumble(CommandXboxController controller, RumbleType rumble, double value) {
        return Commands.runEnd(() -> controller.getHID().setRumble(rumble, value),
            () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0));
    }

    public static Command shutOffRumble(CommandXboxController controller) {
        return Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0));
    }
}
