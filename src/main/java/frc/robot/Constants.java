// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double CAMERA_HEIGHT_METERS = 0.72;
  public static final double TARGET_HEIGHT_METERS = 0;
  public static final double CAMERA_PITCH_RADIANS = 0;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double deadBand = 0.18;
  }

  public static class MotorControllerPorts {
    public static final int kDriveLeft1 = 3;
    public static final int kDriveLeft2 = 4;
    public static final int kDriveRight1 = 1;
    public static final int kDriveRight2 = 2;
  }

  public static class PIDConstants {
    public static final double cycleTime = 50.0;

    public static final double kDriveP = 0.0075;
    public static final double kDriveI = 0.0;
  }

  public static final double tankDriveSpeed = 0.3d; //0.8d

  public static final double driveMinCommand = 0.1;

  public static class FieldConstants {
    public static final double length = Units.feetToMeters(54);
    public static final double width = Units.feetToMeters(27);
  } 

  public static class VisionConstants {
    public static final Transform3d robotToCam =
            new Transform3d(
                   new Translation3d(0.5, 0.0, 0.5),
                    new Rotation3d(
                            0, 0,
                            0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final String cameraName = "5417camera";
    public static final double maxDistanceAway = 2.0;
    public static final double forwardKP = 0.1;
    public static final double forwardToAngleRatio = 0.5;

}
}
