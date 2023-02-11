// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.NavXGyro;
import com.kauailabs.navx.frc.AHRS;

public class NavXGyroCommand extends CommandBase {
  /** Creates a new NavXGyroCommand. */

  NavXGyro m_NavXGyro;
  AHRS ahrs;

  public NavXGyroCommand(NavXGyro m_NavXGyro, AHRS ahrs_passed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_NavXGyro);
    ahrs = ahrs_passed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_NavXGyro = new NavXGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_NavXGyro.printGyro(ahrs);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
