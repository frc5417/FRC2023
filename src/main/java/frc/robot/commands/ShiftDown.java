// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

public class ShiftDown extends CommandBase {
  private final Drive driveSubsystem;
  /** Creates a new ShiftDrivetrain. */
  public ShiftDown(Drive subsystem) {
    driveSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("shifting down");
    driveSubsystem.shiftDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.initTeleopCommand(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
