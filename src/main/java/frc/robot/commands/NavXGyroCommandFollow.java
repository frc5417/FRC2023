// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.PhotonSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants;
import org.photonvision.PhotonCamera;

public class NavXGyroCommandFollow extends CommandBase {
  /** Creates a new NavXGyroCommand. */

  NavXGyro m_NavXGyro;
  AHRS ahrs;
  Drive drive;
  public PhotonSubsystem pcw;
  public PhotonCamera camera;

  private static final double kP = 0.2;
  private static final double kI = 0.0;
  private static final double kD = 3.0;

  private static final PIDController pid = new PIDController(kP, kI, kD);

  public double setAnglePassed;
  public int counter = 0;

  public NavXGyroCommandFollow(NavXGyro m_NavXGyro_passed, AHRS ahrs_passed, Drive drive_passed, PhotonSubsystem pcw_passed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_NavXGyro = m_NavXGyro_passed;
    ahrs = ahrs_passed;
    drive = drive_passed;
    pcw =  pcw_passed;
    addRequirements(m_NavXGyro, drive, pcw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_NavXGyro = new NavXGyro();
    camera = pcw.PhotonCameraWrapper();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_NavXGyro.printGyro(ahrs);
    // while (!this.isFinished()) {
      // turn2Angle(drive, ahrs);
      // System.out.println("THIS FINISHED NOT");
    // };
    // turn2Angle(drive, ahrs);
    // this.cancel();
    if (this.setAnglePassed != 0 && !pid.atSetpoint()) {
      if (Math.abs(Math.abs(m_NavXGyro.getGyroAngle(ahrs)) - Math.abs(this.setAnglePassed)) < 2.5) {
        drive.setPower(0, 0);
        this.setAngle(0);
        System.out.println("STOP");
      }
      double leftPower = pid.calculate(m_NavXGyro.getGyroAngle(ahrs));
      double rightPower = pid.calculate(m_NavXGyro.getGyroAngle(ahrs));
      
      drive.setPower(-MathUtil.clamp(leftPower, -0.8, 0.8), MathUtil.clamp(rightPower, -0.8, 0.8));
    } 









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
    // else {
    //   // if (pid.atSetpoint()) {
    //   //   System.out.printf("P error: %f, V error: %f\n", pid.getPositionError(), pid.getVelocityError());
    //   // }
    //   drive.setPower(0, 0);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
/* 
  public void turn2Angle(Drive drive, AHRS ahrs) {
    if (this.setAnglePassed == 0) return;
    // System.out.printf("Got value %f\n", this.setAnglePassed);
    ahrs.reset();
    while (!pid.atSetpoint()) {
      double leftPower = pid.calculate(m_NavXGyro.getGyroAngle(ahrs))/Math.abs(this.setAnglePassed);
      double rightPower = pid.calculate(m_NavXGyro.getGyroAngle(ahrs))/Math.abs(this.setAnglePassed);
      
      drive.setPower(-MathUtil.clamp(leftPower, -0.8, 0.8), MathUtil.clamp(rightPower, -0.8, 0.8));
      
      // System.out.println("In LOOP!!! :(" + " " + this.setAnglePassed);

      // System.out.println(m_NavXGyro.getGyroAngle(ahrs));
      // System.out.println(pid.atSetpoint());
    }
    drive.setPower(0, 0);
    // System.out.println("Droppped out of loop");
  }*/
  public void setAngle(double set_point) {
      m_NavXGyro.resetGyroAngle(ahrs);
      this.setAnglePassed = set_point*0.1 + this.setAnglePassed*0.9;
      // System.out.printf("Setpoint inside PID %f \n", set_point);
      pid.setSetpoint(this.setAnglePassed);
      pid.setTolerance(5, 1); // stops at <= 25 deg/s error
      // pid.enableContinuousInput(-90, 90);

  }
}
