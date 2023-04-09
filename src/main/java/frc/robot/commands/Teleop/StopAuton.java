// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.*;

public class StopAuton extends CommandBase {
  Drive drive;

  public StopAuton(Drive drive) {
    drive.SetSpeed(0,0);
  }
}