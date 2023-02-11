// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

public class NavXGyro extends SubsystemBase {
  /** Creates a new NavXGyro. */
  public NavXGyro() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void printGyro(AHRS ahrs) {
    System.out.print("NavX Gyro: ");
    System.out.println(ahrs.getAngle());
  }
  public double getGyroAngle(AHRS ahrs) {
    return ahrs.getAngle();
  }
}
