// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.PhotonSubsystem;
import org.photonvision.PhotonCamera;

public class PhotonCommand extends CommandBase {
  /** Creates a new PhotonCommand. */
  public PhotonSubsystem pcw;
  public PhotonCamera camera;

  public int counter = 0;

  public PhotonCommand(PhotonSubsystem photon) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(photon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PHOTON_COMMAND_INIT");
    pcw = new PhotonSubsystem();
    camera = pcw.PhotonCameraWrapper();
    // ahrs.reset();
    // pcw.getEstimatedGlobalPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter++ > 10) {
      pcw.getPose(camera);
      counter = 0;
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
