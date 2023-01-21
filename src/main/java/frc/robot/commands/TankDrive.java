// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive; 

public class TankDrive extends CommandBase {
  private final Drive drive;
  /** Creates a new TankDrive. */
  public TankDrive(Drive subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = subsystem;
    addRequirements(drive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.SetPower(Robot.robotContainer.leftSpeed(), Robot.robotContainer.rightSpeed());
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
