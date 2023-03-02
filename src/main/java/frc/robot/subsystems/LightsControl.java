// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsControl extends SubsystemBase {

  private static DigitalOutput digitalOutput1;
  private static DigitalOutput digitalOutput2;

  /** Creates a new LightsControl. */
  public LightsControl() {
    digitalOutput1 = new DigitalOutput(2);
    digitalOutput2 = new DigitalOutput(3);

    digitalOutput1.set(false);
    digitalOutput2.set(false);
  }

  public void setLightConfig(int configNum) {
    if (configNum == 0) {
      digitalOutput1.set(false);
      digitalOutput2.set(false);
    } else if (configNum == 1) {
      digitalOutput1.set(true); 
      digitalOutput2.set(false);
    } else if (configNum == 2) {
      digitalOutput1.set(false); 
      digitalOutput2.set(true);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
