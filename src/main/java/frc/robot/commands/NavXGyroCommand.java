// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.PhotonSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import org.photonvision.PhotonCamera;

public class NavXGyroCommand extends CommandBase {
  /** Creates a new NavXGyroCommand. */

  NavXGyro m_NavXGyro;
  AHRS ahrs;
  Drive drive;
  public PhotonSubsystem pcw;
  public PhotonCamera camera;

  private static final double kP = 0.8;
  private static final double kI = 0.0;
  private static final double kD = 0.6;

  private static final PIDController pid = new PIDController(kP, kI, kD);

  public double setAnglePassed = 0;
  public int counter = 0;

  private boolean finished = false;

  public NavXGyroCommand(NavXGyro m_NavXGyro_passed, AHRS ahrs_passed, Drive drive_passed, PhotonSubsystem pcw_passed) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("onTrue WOrks");
    if (this.setAnglePassed != 0 && !pid.atSetpoint()) {
      // System.out.println("In if statement");
      if((counter++ % 10) == 0) {
        System.out.println(m_NavXGyro.getGyroAngle());
      }
      if (Math.abs(Math.abs(m_NavXGyro.getGyroAngle()) - Math.abs(this.setAnglePassed)) < 2.5) {
        drive.setPower(0, 0);
        this.setAngle(0);
        System.out.println("STOP");
      }
      double leftPower = pid.calculate(m_NavXGyro.getGyroAngle());
      double rightPower = pid.calculate(m_NavXGyro.getGyroAngle());
      
      drive.setPower(-MathUtil.clamp(leftPower, -0.8, 0.8), MathUtil.clamp(rightPower, -0.8, 0.8));
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ENDGAME");
    drive.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

  public void setAngle(double set_point) {
      m_NavXGyro.resetGyro();
      this.setAnglePassed = set_point;
      pid.setSetpoint(this.setAnglePassed);
      pid.setTolerance(5, 1); // stops at <= 25 deg/s error
      pid.enableContinuousInput(-90, 90);

  }
}