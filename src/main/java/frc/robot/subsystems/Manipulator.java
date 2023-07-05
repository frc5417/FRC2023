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
  private double oldAmperage = 0;
  private final static CANSparkMax manipulatorMotor = new CANSparkMax(ManipulatorConstants.manipulatorPort, MotorType.kBrushless);
  private final static DigitalInput manipulatorSwitch = new DigitalInput(ManipulatorConstants.intakeLimitPort);

  /** Creates a new Manipulator. */
  public Manipulator() {
    manipulatorMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setIntake(double speed) {
    manipulatorMotor.set(0.8 * speed); 
  }

  @Override
  public void periodic() {

    //Unnecessary Amp limit on intake
    /*    
    if (((manipulatorMotor.getOutputCurrent() + oldAmperage)/2) > 40){
      manipulatorMotor.set(0);
    }

    oldAmperage = manipulatorMotor.getOutputCurrent();
  */    
  }
  
  public boolean cancelIfLimitTriggered() {
    if(!manipulatorSwitch.get()){
      //manipulatorMotor.set(0);
      return true;
    }
    return false;
  }
}