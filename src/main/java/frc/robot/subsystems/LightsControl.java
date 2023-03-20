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

  private static int config = 0;
  private static int animFrame = 0;
  private static int animDir = 1;
  private static int animStage = 1;
  private static int skipFrame = 0;

  private static AddressableLEDBuffer ledBuffer;

  /** Creates a new LightsControl. */
  public LightsControl() {
    led = new AddressableLED(9);
    // led2 = new AddressableLED(8);

    ledBuffer = new AddressableLEDBuffer(30*2);
    led.setLength(ledBuffer.getLength());
    // led2.setLength(ledBuffer.getLength());

    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 255, 0, 0);
     }

    // Set the data
    led.setData(ledBuffer);
    // led2.setData(ledBuffer);

    led.start();
    // led2.start();
  }

  public void setLightConfig(int configNum) {
    config = configNum;

    if (configNum == 0 || configNum == 4) {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for black
        ledBuffer.setRGB(i, 0, 0, 0);
      }
    } else if (configNum == 1) {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for orange
        ledBuffer.setRGB(i, 255, 191, 0);
      }  
    } else if (configNum == 2) {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for purple
        ledBuffer.setRGB(i, 191, 64, 191);
      }
    } else if (configNum == 3) {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for off
        ledBuffer.setRGB(i, 0, 0, 0);
      }
    }

    led.setData(ledBuffer);
    // led2.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (config == 0 || config == 4) {
      // Animation One: Up and Down
      if (animStage == 0) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
          // Sets the specified LED to the RGB values for black
          ledBuffer.setRGB(i, 0, 0, 0);
        }

        for (var i = 0; i < animFrame; i++) {
          // Sets the specified LED to the RGB values for red
          ledBuffer.setRGB(i, (config == 0 ? 255 : 0), 0, (config == 4 ? 255 : 0));
        }

        animFrame += animDir;
        if(animFrame >= ledBuffer.getLength() || animFrame < 1) {
          animDir *= -1;
        }
      }

      // Animation Two: Moving white sections on background
      else if (animStage == 1) {
        if (skipFrame < 2) {
          skipFrame += 1;
          return;
        }

        skipFrame = 0;
        
        for (var i = 0; i < ledBuffer.getLength(); i++) {
          // Sets the specified LED to the RGB values for red
          ledBuffer.setRGB(i, (config == 0 ? 255 : 0), 0, (config == 4 ? 255 : 0));
        }

        var i = animFrame % 12;
        while (i < ledBuffer.getLength())
        {
          ledBuffer.setRGB(i, 255, 255, 255);
          if (i + 1 < ledBuffer.getLength())
            ledBuffer.setRGB(i + 1, 255, 255, 255);
          i += 12;
        }

        animFrame += 1;
        if (animFrame >= 800)
        {
          animFrame = 0;
        }
      }

      led.setData(ledBuffer);
    }

  }
}