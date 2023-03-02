// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class AutoArmSetPos extends CommandBase {
  private final Arm manipulatorSubsystem;

  private final double setPoint;
  private boolean doFinish = false;
  private int counter = 0;

  public AutoArmSetPos(double pos, Arm subsystem) {
    manipulatorSubsystem = subsystem;

    if (pos < Constants.ManipulatorConstants.minSetPoint) {
      pos = Constants.ManipulatorConstants.minSetPoint;
    }
    
    setPoint = pos;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter++ % 10 == 0) { System.out.println(Math.abs(manipulatorSubsystem.runningAverage - setPoint)); }
    if (Math.abs(manipulatorSubsystem.runningAverage - setPoint) < 0.05) {
      doFinish = true;
    }
    manipulatorSubsystem.setArmPos(setPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulatorSubsystem.setArm(0.0d);
    //RobotContainer.initArmMovement();
    System.out.println("ending set position");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return doFinish;
  }
}