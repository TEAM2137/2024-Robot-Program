package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RumbleSequences {
    public static Command rumbleDualPulse(XboxController controller) {
        return 
            rumble(controller, RumbleType.kBothRumble, 1)
            .andThen(Commands.waitSeconds(.1))
            .andThen(shutOffRumble(controller))
            .andThen(Commands.waitSeconds(.1))
            .andThen(rumble(controller, RumbleType.kBothRumble, 1))
            .andThen(Commands.waitSeconds(.1))
            .andThen(shutOffRumble(controller));
    }

    public static Command rumbleOnce(XboxController controller) {
        return
            rumble(controller, RumbleType.kBothRumble, 1)
            .andThen(Commands.waitSeconds(.2))
            .andThen(shutOffRumble(controller));
    }

    public static Command alternatingRumbles(XboxController controller) {
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

    public static Command rumble(XboxController controller, RumbleType rumble, double value) {
        return new InstantCommand(() -> controller.setRumble(rumble, value));
    }

    public static Command shutOffRumble(XboxController controller) {
        return new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble, 0));
    }
}
