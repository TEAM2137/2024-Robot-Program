package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDrivetrain;

public class Robot extends TimedRobot {
  private String selectedAutonName;
  private final SendableChooser<String> autonChooser = new SendableChooser<>();

  public SwerveDrivetrain swerveDrivetrain;
  public Autonomous autonomous;

  /** This function is run when the robot is first started up. */
  @Override
  public void robotInit() {
    swerveDrivetrain = new SwerveDrivetrain();
    autonomous = new Autonomous();
    autonChooser.setDefaultOption("Default Auton", "default-auton");
    SmartDashboard.putData("AutoChooser", autonChooser);
  }

  /** This function is called periodically during any mode. */
  @Override
  public void robotPeriodic() {}

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {
    selectedAutonName = autonChooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + selectedAutonName);

    autonomous.init();

    AutoBuilder.configureHolonomic(
      null, null, // Get and reset the pose of the robot
      null, null, // Move the robot during auton
      new HolonomicPathFollowerConfig( // Config
        new PIDConstants(0, 0, 0),
        new PIDConstants(0, 0, 0),
        kDefaultPeriod, // Max module speed
        kDefaultPeriod, // Radius of the robot base
        new ReplanningConfig()
      ),
      null // Swerve subsystem
    );
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

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
