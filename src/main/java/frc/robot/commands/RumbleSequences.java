package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleSequences {
    public static Command rumbleDualPulse(CommandXboxController controller) {
        return 
            rumble(controller, RumbleType.kBothRumble, 1)
            .andThen(Commands.waitSeconds(.1))
            .andThen(shutOffRumble(controller))
            .andThen(Commands.waitSeconds(.1))
            .andThen(rumble(controller, RumbleType.kBothRumble, 1))
            .andThen(Commands.waitSeconds(.1))
            .andThen(shutOffRumble(controller));
    }

    public static Command rumbleOnce(CommandXboxController driverController) {
        return
            rumble(driverController, RumbleType.kBothRumble, 1)
            .andThen(Commands.waitSeconds(.2))
            .andThen(shutOffRumble(driverController));
    }

    public static Command alternatingRumbles(CommandXboxController controller) {
        return // This is incredibly ugly but when i tried making it pretty it crashed the robot
            rumble(controller, RumbleType.kLeftRumble, 1)
            .andThen(Commands.waitSeconds(.1))
            .andThen(rumble(controller, RumbleType.kRightRumble, 1).alongWith(rumble(controller, RumbleType.kLeftRumble, 0)))
            .andThen(Commands.waitSeconds(.1))
            .andThen(rumble(controller, RumbleType.kRightRumble, 0).alongWith(rumble(controller, RumbleType.kLeftRumble, 1)))
            .andThen(Commands.waitSeconds(.1))
            .andThen(rumble(controller, RumbleType.kRightRumble, 1).alongWith(rumble(controller, RumbleType.kLeftRumble, 0)))
            .andThen(shutOffRumble(controller));
    }

    public static Command rumble(CommandXboxController controller, RumbleType rumble, double value) {
        return new InstantCommand(() -> controller.getHID().setRumble(rumble, value));
    }

    public static Command shutOffRumble(CommandXboxController controller) {
        return new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0));
    }
}
