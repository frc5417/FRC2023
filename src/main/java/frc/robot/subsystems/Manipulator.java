// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
  private final static CANSparkMax manipulatorMotor = new CANSparkMax(ManipulatorConstants.manipulatorPort, MotorType.kBrushless);
  private final static DigitalInput manipulatorSwitch = new DigitalInput(ManipulatorConstants.intakeLimitPort);
  private final static DoubleSolenoid claw1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, ManipulatorConstants.kClaw1Solenoid, ManipulatorConstants.kClaw2Solenoid);
  private final static DoubleSolenoid claw2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, ManipulatorConstants.kClaw3Solenoid, ManipulatorConstants.kClaw4Solenoid);

  /** Creates a new Manipulator. */
  public Manipulator() {
    manipulatorMotor.setIdleMode(IdleMode.kBrake);

    claw1.set(DoubleSolenoid.Value.kReverse);
    claw2.set(DoubleSolenoid.Value.kReverse);
  }

  public void setIntake(double speed) {
    manipulatorMotor.set(speed); 
  }

  public void setClaw(int configType) {
    if (configType == 1) {
      claw1.set(DoubleSolenoid.Value.kForward);
      claw2.set(DoubleSolenoid.Value.kReverse);
    } else if (configType == 2) {
      claw1.set(DoubleSolenoid.Value.kForward);
      claw2.set(DoubleSolenoid.Value.kForward);
    } if (configType == 3) {
      claw1.set(DoubleSolenoid.Value.kReverse);
      claw2.set(DoubleSolenoid.Value.kForward);
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