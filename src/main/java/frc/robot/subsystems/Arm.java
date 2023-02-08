// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private final CANSparkMax armMotor1;
  private final CANSparkMax armMotor2;

  private final static DutyCycleEncoder enc = new DutyCycleEncoder(Constants.ManipulatorConstants.armEncoderPort);
  
  private double voltage = 0.0;
  private double integral = 0.0;
  private double derivative = 0.0;
  private double oldError = 0.0;

  /** Creates a new Arm. */
  public Arm() {
    armMotor1 = new CANSparkMax(Constants.ManipulatorConstants.armLeaderPort, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(Constants.ManipulatorConstants.armFollowerPort, MotorType.kBrushless);

    armMotor1.setIdleMode(IdleMode.kBrake);
    armMotor2.setIdleMode(IdleMode.kBrake);

    armMotor2.follow(armMotor1);
  }

  public void setArm(double speed) {
    armMotor1.set(speed);
  }

  public void setArmPos(double pos) {
    armMotor1.setVoltage(PID(pos));
  }

  public double PID(double setPoint) {
    // System.out.println(setPoint + ", " + enc.getAbsolutePosition() + ", " + (setPoint - enc.getAbsolutePosition()));

    double error = setPoint - enc.getAbsolutePosition();
    double proportional = error * Constants.ManipulatorConstants.kArmP;
    integral += error * Constants.ManipulatorConstants.kArmI * Constants.ManipulatorConstants.cycleTime;
    derivative = Constants.ManipulatorConstants.kArmD * (error - oldError) / Constants.ManipulatorConstants.cycleTime;

    voltage += proportional + integral + derivative;

    oldError = error;
    // makes the lower limit -3.0 and upper 1 
    if(voltage < -Constants.ManipulatorConstants.maxVoltage) {
      voltage = -Constants.ManipulatorConstants.maxVoltage;
    } else if (voltage > 0) {
      voltage = 0;
    }

    return voltage;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
