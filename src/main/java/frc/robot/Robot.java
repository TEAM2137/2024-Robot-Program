package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This is the main robot class. It shouldn't need to be modified
 * unless absolutely necessary.
 */
public class Robot extends TimedRobot {
  public RobotContainer robotContainer;
  
  @Override public void robotInit() {
    try { Thread.sleep(250); }
    catch(Exception e) {}
    robotContainer = new RobotContainer();
  }
  @Override public void robotPeriodic() { CommandScheduler.getInstance().run(); }
  @Override public void autonomousInit() { robotContainer.runAutonomous(); }
  @Override public void autonomousPeriodic() { robotContainer.auto.autonomousPeriodic(); }
  @Override public void teleopInit() { robotContainer.runTeleop(); }
  @Override public void teleopPeriodic() {}
  @Override public void disabledInit() {}
  @Override public void disabledPeriodic() { robotContainer.disabledPeriodic(); }
  @Override public void testInit() {}
  @Override public void testPeriodic() {}
  @Override public void simulationInit() {}
  @Override public void simulationPeriodic() {}
}
