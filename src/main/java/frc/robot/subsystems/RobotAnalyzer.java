// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotAnalyzer extends SubsystemBase {

  private static AHRS acceleromoter = new AHRS();

  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("robotAnalyzer");

  /** Creates a new RobotAnalyzer. */
  public RobotAnalyzer() {}

  @Override
  public void periodic() {
    table.getEntry("velocityX").setDouble(acceleromoter.getVelocityX());
    table.getEntry("velocityY").setDouble(acceleromoter.getVelocityY());
    table.getEntry("velocityZ").setDouble(acceleromoter.getVelocityZ());

    table.getEntry("accelX").setDouble(acceleromoter.getRawAccelX());
    table.getEntry("accelY").setDouble(acceleromoter.getRawAccelY());
    table.getEntry("accelZ").setDouble(acceleromoter.getRawAccelZ());
  }
}
