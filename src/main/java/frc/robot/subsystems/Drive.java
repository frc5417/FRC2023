// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  AHRS ahrs;
  CANSparkMax DriveLeftMaster = new CANSparkMax(Constants.DriveLeftMaster, MotorType.kBrushless);
  CANSparkMax DriveLeftSlave = new CANSparkMax(Constants.DriveLeftSlave, MotorType.kBrushless);
  CANSparkMax DriveRightMaster = new CANSparkMax(Constants.DriveRightMaster, MotorType.kBrushless);
  CANSparkMax DriveRightSlave = new CANSparkMax(Constants.DriveRightSlave, MotorType.kBrushless);

  Solenoid Shifter = new Solenoid(PneumaticsModuleType.REVPH, 0);
  /** Creates a new Drive. */
  public Drive() {
    DriveRightMaster.setInverted(true);
    DriveLeftMaster.setInverted(false);

    DriveRightMaster.getPIDController().setP(Constants.DriverConstants.kDriveP);
    DriveLeftMaster.getPIDController().setP(Constants.DriverConstants.kDriveP);

    DriveLeftSlave.follow(DriveLeftMaster);
    DriveRightSlave.follow(DriveRightMaster);
  }
  public double GyroPitch(){
    return ahrs.getPitch();
  }

  public void SetSpeed(double leftSpeed, double rightSpeed) {
    DriveLeftMaster.set(leftSpeed);
    DriveRightMaster.set(rightSpeed);
    DriveLeftSlave.follow(DriveLeftMaster);
    DriveRightSlave.follow(DriveRightMaster);
  }

  public void AutoPower(double leftPower, double rightPower) {
    DriveLeftMaster.set(leftPower);
    DriveRightMaster.set(rightPower);
  }

  public void ShiftToggle() {
    Shifter.set(!Shifter.get());
  }

  public void balance(){
    while(GyroPitch() >= Constants.degreesAllowed){
      AutoPower(-0.1, -0.1);
    }

    while(GyroPitch() <= -(Constants.degreesAllowed)){
      AutoPower(0.1, 0.1);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
