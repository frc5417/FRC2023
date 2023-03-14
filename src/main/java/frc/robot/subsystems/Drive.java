// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  private final static CANSparkMax leftmotors1 = new CANSparkMax(Constants.MotorControllerPorts.kDriveLeft1, MotorType.kBrushless);
  private final static CANSparkMax leftmotors2 = new CANSparkMax(Constants.MotorControllerPorts.kDriveLeft2, MotorType.kBrushless);
  private final static CANSparkMax rightmotors1 = new CANSparkMax(Constants.MotorControllerPorts.kDriveRight1, MotorType.kBrushless);
  private final static CANSparkMax rightmotors2 = new CANSparkMax(Constants.MotorControllerPorts.kDriveRight2, MotorType.kBrushless);
  
  private final static MotorControllerGroup leftMotors = new MotorControllerGroup(leftmotors1, leftmotors2);
  private final static MotorControllerGroup rightMotors = new MotorControllerGroup(rightmotors1, rightmotors2);

  public Drive() {
    leftMotors.setInverted(false);
    rightMotors.setInverted(false);
    
    leftmotors1.setIdleMode(IdleMode.kCoast);
    leftmotors2.setIdleMode(IdleMode.kCoast);
    rightmotors1.setIdleMode(IdleMode.kCoast);
    rightmotors2.setIdleMode(IdleMode.kCoast);
  }
  
  public void setPower(double leftPower, double rightPower) {
    if (Math.abs(leftPower) < Constants.OperatorConstants.deadBand) {
      leftPower = 0;
    } else if (leftPower > Constants.tankDriveSpeed) { 
      leftPower = Constants.tankDriveSpeed; 
    }

    if (Math.abs(rightPower) < Constants.OperatorConstants.deadBand) {
      rightPower = 0;
    } else if (rightPower > Constants.tankDriveSpeed) { 
      rightPower = Constants.tankDriveSpeed; 
    }

    leftMotors.set(leftPower * Constants.tankDriveSpeed);
    rightMotors.set(rightPower * Constants.tankDriveSpeed);
  }

  @Override
  public void periodic() {}

}