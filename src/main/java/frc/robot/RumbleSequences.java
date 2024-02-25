package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RumbleSequences {
    public static Command rumbleDualPulse(XboxController controller) {
        return 
            rumble(controller, RumbleType.kBothRumble, 1)
            .andThen(CommandSequences.timingCommand(.1))
            .andThen(rumble(controller, RumbleType.kBothRumble, 0))
            .andThen(CommandSequences.timingCommand(.1))
            .andThen(rumble(controller, RumbleType.kBothRumble, 1))
            .andThen(CommandSequences.timingCommand(.1))
            .andThen(rumble(controller, RumbleType.kBothRumble, 0));
    }

    public static Command rumbleOnce(XboxController controller) {
        return
            rumble(controller, RumbleType.kBothRumble, 1)
            .andThen(CommandSequences.timingCommand(.2))
            .andThen(rumble(controller, RumbleType.kBothRumble, 0));
    }

    public static Command alternatingRumbles(XboxController controller) {
        return
            rumble(controller, RumbleType.kLeftRumble, 1)
            .andThen(CommandSequences.timingCommand(.1))
            .andThen(rumble(controller, RumbleType.kRightRumble, 1).alongWith(rumble(controller, RumbleType.kLeftRumble, 0)))
            .andThen(CommandSequences.timingCommand(.1))
            .andThen(rumble(controller, RumbleType.kRightRumble, 0).alongWith(rumble(controller, RumbleType.kLeftRumble, 1)))
            .andThen(CommandSequences.timingCommand(.1))
            .andThen(rumble(controller, RumbleType.kRightRumble, 1).alongWith(rumble(controller, RumbleType.kLeftRumble, 0)))
            .andThen(rumble(controller, RumbleType.kBothRumble, 0));
    }

    public static Command rumble(XboxController controller, RumbleType rumble, double value) {
        return new InstantCommand(() -> controller.setRumble(rumble, value));
    }
}
