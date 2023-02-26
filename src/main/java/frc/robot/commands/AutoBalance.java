// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalance extends CommandBase {
  private final static Drive drive = new Drive();
  /** Creates a new AutoBalance. */

  //private static Boolean doFinish = false;
  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //doFinish = 
    drive.balance();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.SetSpeed(0,0);
    RobotContainer.initTeleopCommand();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
