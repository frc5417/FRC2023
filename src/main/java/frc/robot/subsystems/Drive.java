// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  CANSparkMax DriveLeftMaster = new CANSparkMax(Constants.DriveLeftMaster, MotorType.kBrushless);
  CANSparkMax DriveLeftSlave = new CANSparkMax(Constants.DriveLeftSlave, MotorType.kBrushless);
  CANSparkMax DriveRightMaster = new CANSparkMax(Constants.DriveRightMaster, MotorType.kBrushless);
  CANSparkMax DriveRightSlave = new CANSparkMax(Constants.DriveRightSlave, MotorType.kBrushless);
  Solenoid Shifter = new Solenoid(null, 0);


  int toggle;
  int count;

  /** Creates a new Drive. */
  public Drive() {
    DriveRightMaster.setInverted(true);
    DriveLeftMaster.setInverted(false);
    DriveLeftSlave.follow(DriveLeftMaster);
    DriveRightSlave.follow(DriveRightMaster);
  }

  public void SetPower(double leftPower, double rightPower) {
    DriveLeftMaster.set(Math.pow(leftPower, 3));
    DriveRightMaster.set(Math.pow(rightPower, 3));
    DriveLeftSlave.follow(DriveLeftMaster);
    DriveRightSlave.follow(DriveRightMaster);
  }

  public void AutoPower(double leftPower, double rightPower) {
    DriveLeftMaster.set(leftPower);
    DriveRightMaster.set(rightPower);
  }

  public void Shift(boolean button) {
    if (button) {
      count += 1;
    }

    switch(count) {
      case 0:
        toggle = 0;
        break;
      case 1:
        toggle = 1;
        break;
      case 2:
        toggle = 0;
        count = 0;
        break;
    }

    switch(toggle) {
      case 0:
        Shifter.set(false);
        break;
      case 1:
        Shifter.set(true);
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
