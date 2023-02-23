// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import org.photonvision.PhotonCamera;

public class PhotonDirectDrive extends CommandBase {
  /** Creates a new PhotonCommand. */
  public PhotonSubsystem pcw;
  public Drive drive;
  public PhotonCamera camera;

  public int counter = 0;

  public PhotonDirectDrive(PhotonSubsystem photon, Drive drivetrain) {
    pcw = photon;
    drive = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pcw);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera = pcw.PhotonCameraWrapper();
    
    System.out.println("PHOTON DRIVE INIT");
    // ahrs.reset();
    // pcw.getEstimatedGlobalPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  //Basic code to make the robot go up to maxDistanceAway from the target in a straight line
  @Override
  public void execute() {
    
    double leftCommand = 0.0;
    double rightCommand = 0.0;
    //gets the difference in the current distance away from what it should be
    double forwardError = pcw.getDistance(camera) - Constants.VisionConstants.maxDistanceAway;
    double angleError = pcw.getYaw(camera);
    if (forwardError > 0.0) {
      //if positive forward error, then we need to correct it by driving more forwards
      leftCommand = 1.0;
      rightCommand = 1.0;
    }
    if (angleError > 0.0) {
      //if positive angle error (rotated clockwise of target), then we need to correct it by turning counterclockwise
      leftCommand = Constants.VisionConstants.forwardToAngleRatio + (angleError/90.0)*(1-Constants.VisionConstants.forwardToAngleRatio);
      rightCommand = Constants.VisionConstants.forwardToAngleRatio - (angleError/90.0)*(1-Constants.VisionConstants.forwardToAngleRatio);
    }

    if ((counter++ % 3) == 0){
      System.out.printf("F_Err: %f, A_Err %f \n", forwardError, angleError);
      System.out.printf("Left: %f, Right %f \n", leftCommand*Constants.VisionConstants.forwardKP, rightCommand*Constants.VisionConstants.forwardKP);
    }

    
    drive.setPower(Math.abs(leftCommand*0.5), Math.abs(rightCommand*0.5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setPower(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
