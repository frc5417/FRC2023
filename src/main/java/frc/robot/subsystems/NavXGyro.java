// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

public class NavXGyro extends SubsystemBase {
  public AHRS ahrs; 

  public NavXGyro(AHRS gyro) {
    ahrs = gyro;
  }

  public double getGyroAngle() {
    return ahrs.getAngle();
  }

  public void resetGyro() {
    ahrs.reset();
  }

}