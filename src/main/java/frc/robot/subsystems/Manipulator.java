// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
  private final static CANSparkMax manipulatorMotor = new CANSparkMax(ManipulatorConstants.manipulatorPort, MotorType.kBrushless);
  private final static DigitalInput manipulatorSwitch = new DigitalInput(ManipulatorConstants.intakeLimitPort);
  private final static Solenoid claw1 = new Solenoid(PneumaticsModuleType.REVPH, ManipulatorConstants.kClaw1Solenoid);
  private final static Solenoid claw2 = new Solenoid(PneumaticsModuleType.REVPH, ManipulatorConstants.kClaw2Solenoid);
  private final static Solenoid claw3 = new Solenoid(PneumaticsModuleType.REVPH, ManipulatorConstants.kClaw3Solenoid);

  /** Creates a new Manipulator. */
  public Manipulator() {
    manipulatorMotor.setIdleMode(IdleMode.kBrake);

    claw1.set(false);
    claw2.set(false);
    claw3.set(false);
  }

  public void setIntake(double speed) {
    manipulatorMotor.set(speed); 
  }

  public void setClaw(){
    claw1.set(!claw1.get());
    claw2.set(!claw2.get());
  }

  public void setClaw(int configType) {
    if (configType == 1) {
      claw1.set(true);
      claw2.set(false);
      claw3.set(true);
    } else if (configType == 2) {
      claw1.set(true);
      claw2.set(true);
      claw3.set(false);
    } if (configType == 3) {
      claw1.set(false);
      claw2.set(false);
      claw3.set(true);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public boolean cancelIfLimitTriggered() {
    if(!manipulatorSwitch.get()){
      manipulatorMotor.set(0);
      return true;
    }
    return false;
  }
}