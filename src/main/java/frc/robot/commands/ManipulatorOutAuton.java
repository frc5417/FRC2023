// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.Manipulator;

public class ManipulatorOutAuton extends CommandBase {
  private final Manipulator manipulatorSubsystem;
  private int codeSleep;
  private int count = 0;

  private boolean doFinish = false;

  /** Creates a new ManipulatorToggle. */
  public ManipulatorOutAuton(Manipulator subsystem, int sleepTime) {
    manipulatorSubsystem = subsystem;
    codeSleep = sleepTime;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    manipulatorSubsystem.setIntake(-ManipulatorConstants.manipulatorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (count++ >= (codeSleep / 20)) {
      doFinish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    manipulatorSubsystem.setIntake(0);
  }

  @Override
  public boolean isFinished() {
    return doFinish;
  }
}