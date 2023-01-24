// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.vision;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** An example command that uses an example subsystem. */
public class visionMove extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final vision m_limelight;
  private final double steerMultiplier = 0.5;
  private final double driveMultiplier = 0.5;
  private final double desiredArea = 25;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public visionMove(vision vision) {
    m_limelight = vision;

    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_limelight.getV()){
      SmartDashboard.putNumber("Steering", m_limelight.getX() * steerMultiplier);
      SmartDashboard.putNumber("Driving", (desiredArea - m_limelight.getArea()) * driveMultiplier );
    }
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
