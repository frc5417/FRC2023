// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
//import all the modules needed
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  private final static CANSparkMax leftmotors1 = new CANSparkMax(Constants.MotorControllerPorts.kDriveLeft1, MotorType.kBrushless);
  private final static CANSparkMax leftmotors2 = new CANSparkMax(Constants.MotorControllerPorts.kDriveLeft2, MotorType.kBrushless);
  private final static CANSparkMax rightmotors1 = new CANSparkMax(Constants.MotorControllerPorts.kDriveRight1, MotorType.kBrushless);
  private final static CANSparkMax rightmotors2 = new CANSparkMax(Constants.MotorControllerPorts.kDriveRight2, MotorType.kBrushless);
  
  public Drive() {
    leftmotors1.setInverted(true);
    leftmotors2.setInverted(true);
    
    leftmotors1.setIdleMode(IdleMode.kCoast);
    leftmotors2.setIdleMode(IdleMode.kCoast);
    rightmotors1.setIdleMode(IdleMode.kCoast);
    rightmotors2.setIdleMode(IdleMode.kCoast);
  }
  
  public void setPower(double leftPower, double rightPower) {

    leftPower *= -1;
    rightPower *= -1;

    if (Math.abs(leftPower) < Constants.OperatorConstants.deadBand) {
      leftPower = 0;
    }

    if (Math.abs(rightPower) < Constants.OperatorConstants.deadBand) {
      rightPower = 0;
    }

    // System.out.println(leftPower + ", " + rightPower);

    if (leftPower > Constants.tankDriveSpeed) { leftPower = Constants.tankDriveSpeed; }
    if (rightPower > Constants.tankDriveSpeed) { rightPower = Constants.tankDriveSpeed; }

    leftmotors1.set(leftPower * Constants.tankDriveSpeed);
    leftmotors2.set(leftPower * Constants.tankDriveSpeed);

    rightmotors1.set(rightPower * Constants.tankDriveSpeed);
    rightmotors2.set(rightPower * Constants.tankDriveSpeed);
  }

  @Override
  public void periodic() {}

}
