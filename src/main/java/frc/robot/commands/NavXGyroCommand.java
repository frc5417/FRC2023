// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavXGyro;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class NavXGyroCommand extends CommandBase {
  /** Creates a new NavXGyroCommand. */

  NavXGyro m_NavXGyro;
  AHRS ahrs;
  Drive drive;

  private static final double kP = 0.8;
  private static final double kI = 0.1;
  private static final double kD = 0.4;

  private static final PIDController pid = new PIDController(kP, kI, kD);

  public double setAnglePassed;

  public NavXGyroCommand(NavXGyro m_NavXGyro_passed, AHRS ahrs_passed, Drive drive_passed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_NavXGyro = m_NavXGyro_passed;
    ahrs = ahrs_passed;
    drive = drive_passed;
    addRequirements(m_NavXGyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_NavXGyro = new NavXGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_NavXGyro.printGyro(ahrs);
    // while (!this.isFinished()) {
      turn2Angle(drive, ahrs);
      // System.out.println("THIS FINISHED NOT");
    // };
    // turn2Angle(drive, ahrs);
    // this.cancel();
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
  }
  public void setAngle(double set_point) {
    // System.out.printf("Set to: %f\n", set_point);
    this.setAnglePassed = set_point;
    pid.setSetpoint(this.setAnglePassed);
    pid.setTolerance(0.01, 0);
    // turn2Angle(drive, ahrs);
  }
}
