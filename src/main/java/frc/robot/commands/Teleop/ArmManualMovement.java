// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.Arm;

public class ArmManualMovement extends CommandBase {
  private final Arm armSubystem;

  /** Creates a new ArmManualMovement. */
  public ArmManualMovement(Arm subsystem) {
    armSubystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftJoystick = RobotContainer.getManipulatorLeftJoystick() * ManipulatorConstants.armMaxSpeed;
    armSubystem.setArm(leftJoystick);
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
