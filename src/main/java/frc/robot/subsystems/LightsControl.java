// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsControl extends SubsystemBase {

  private static AddressableLED led;
  // private static AddressableLED led2;

  private static int config = 0;
  private static int animFrame = 0;
  private static int animDir = 1;
  private static int animRunCount = 0;
  private static int animStage = 2;
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
        if (animFrame >= 800) {
          animFrame = 0;
        }
      }

      else if (animStage == 2) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          int r = getR(ledBuffer, i);
          int g = getG(ledBuffer, i);
          int b = getB(ledBuffer, i);
          if (r >= 10) r -= 10; else r = 0;
          if (g >= 20) g -= 20; else g = 0;
          if (b >= 20) b -= 20; else b = 0;
          ledBuffer.setRGB(i, r, g, b);
        }
        //m_ledBuffer[m_currentPixel].SetTrue_R_G_B(64, 64, 64);
        ledBuffer.setRGB(animFrame, 120, 120, 120);
        // m_ledB.SetData(m_ledBuffer);
        
        animFrame += animDir;
        if ((animFrame <= 0) || (animFrame >= ledBuffer.getLength() - 1)) {
          // Ensure valid even when switching modes
          if (animFrame < 0) animFrame = 0;
          if (animFrame > ledBuffer.getLength() - 1) animFrame = ledBuffer.getLength() - 1;
          animDir = -animDir;
        }
      }

      animRunCount += 1;
      if (animRunCount >= 800) {
        animRunCount = 0;
        animStage += 1;
        if (animStage > 2) {
          animStage = 0;
        }
      }

      led.setData(ledBuffer);
    }

  }

  private int getR(AddressableLEDBuffer m_ledBuffer, int i) {
    return (int) m_ledBuffer.getLED(i).red;
  }
  private int getG(AddressableLEDBuffer m_ledBuffer, int i) {
    return (int) m_ledBuffer.getLED(i).blue;
  }
  private int getB(AddressableLEDBuffer m_ledBuffer, int i) {
    return (int) m_ledBuffer.getLED(i).green;
  }
}