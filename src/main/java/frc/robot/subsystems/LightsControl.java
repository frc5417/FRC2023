// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsControl extends SubsystemBase {

  private static AddressableLED led;
  private static AddressableLED led2;

  private static AddressableLEDBuffer ledBuffer;

  /** Creates a new LightsControl. */
  public LightsControl() {
    led = new AddressableLED(9);
    led2 = new AddressableLED(9);

    ledBuffer = new AddressableLEDBuffer(29*2);
    led.setLength(ledBuffer.getLength());
    led2.setLength(ledBuffer.getLength());

    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 255, 0, 0);
     }     

    // Set the data
    led.setData(ledBuffer);
    led2.setData(ledBuffer);

    led.start();
    led2.start();
  }

  public void setLightConfig(int configNum) {
    if (configNum == 0) {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 255, 0, 0);
     }     
    } else if (configNum == 1) {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 255, 191, 0);
     }  
    } else if (configNum == 2) {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 191, 64, 191);
     }  
    }

    led.setData(ledBuffer);
    led2.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}