// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class AutoArmSetPos extends CommandBase {
  private final Arm armSubsystem;

  private final double setPoint;
  private boolean doFinish = false;
  private int counter = 0;
  private boolean endFlag = false;

  public AutoArmSetPos(double pos, Arm subsystem, boolean eventuallyStop) {
    armSubsystem = subsystem;
    endFlag = eventuallyStop;

    if (pos < Constants.ManipulatorConstants.minSetPoint) {
      pos = Constants.ManipulatorConstants.minSetPoint;
    }
    
    setPoint = pos;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    doFinish = false;
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(armSubsystem.runningAverage - setPoint) < 0.05 && endFlag) {
      counter++;
      if (counter > 15) {
        doFinish = true;
      }
    } else {
      counter = 0;
    }
    
    if (!armSubsystem.setArmPos(setPoint)) doFinish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setArm(0.0d);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return doFinish;
  }
}