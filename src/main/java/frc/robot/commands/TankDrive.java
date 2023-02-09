// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.PhotonSubsystem;


import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final PhotonSubsystem m_photonsubsystem = new PhotonSubsystem();
  private final PhotonCommand m_pPhotonCommand = new PhotonCommand(m_photonsubsystem);

  private final Drive drive;
  public TankDrive(Drive subsystem) {
    drive = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setPower(RobotContainer.getDriverLeftJoystick(), RobotContainer.getDriverRightJoystick());
    if(RobotContainer.getButtonA()) {
      CommandScheduler.getInstance().schedule(m_pPhotonCommand);
    } else {
      m_pPhotonCommand.cancel();
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
