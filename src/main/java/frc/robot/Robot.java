package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  
  public CommandScheduler scheduler;
  public RobotContainer robotContainer;
  
  public Command autonCommand;

  /** This function is run when the robot is first started up. */
  @Override
  public void robotInit() {
    // Gets an instance of the command scheduler
    scheduler = CommandScheduler.getInstance();
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during any mode. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {
    robotContainer.autonomousInit();
    autonCommand = robotContainer.getAutonomousCommand();
    if (autonCommand != null) {
      autonCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // Cancel autonomous in case it is still running for some reason
    if (autonCommand != null) {
      autonCommand.cancel();
    }
    robotContainer.runTeleop();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
