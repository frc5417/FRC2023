// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Team 5417 Robot Code
  private Command m_autonomousCommand;

  public static RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();

    String[] autoList = {"High Score Mobility", "Low Scoring Mobility", "Docking", "Engaging Low Score", "Engaging High Score"};
    SmartDashboard.putStringArray("Auto List", autoList);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotContainer.setLEDsOff();
    RobotContainer.setBrakeMode();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    String autoSelected = SmartDashboard.getString("Auto Selector", "Scoring Auto");
    System.out.println("Auto selected: " + autoSelected);

    switch(autoSelected) {
      case "High Score Mobility":
        m_autonomousCommand = robotContainer.coneScoreAutonomousCommand();
        break;
      case "Docking":
        m_autonomousCommand = robotContainer.dockAutonomousCommand();
        break;
      case "Engaging Low Score":
        m_autonomousCommand = robotContainer.engageAutonomousCommand();
        break;
      case "Engaging High Score":
        m_autonomousCommand = robotContainer.engageScoreAutonomousCommand();
        break;
      case "Low Scoring Mobility":
        m_autonomousCommand = robotContainer.lowMobilityAutonomouCommand();
        break;
      default:
        break;
    }

    RobotContainer.resetOdometry();
    RobotContainer.setCoastMode();
    RobotContainer.setLEDsOn();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    
    RobotContainer.setLEDsOn();
    RobotContainer.setCoastMode();
    RobotContainer.initArmMovement();
    RobotContainer.initTeleopCommand();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

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
