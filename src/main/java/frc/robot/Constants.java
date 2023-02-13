// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int DriveLeftLeader = 11;
  public static final int DriveLeftFollower = 12;
  public static final int DriveRightLeader = 13;
  public static final int DriveRightFollower = 14;
  public static final double degreesAllowed = 2.5;
  public static final double maxVoltage = 10;
  public static final double maxSpeed = 3; // m/s
  public static final double maxAcceleration =  3;//m/s^2
  public static final double trackWidth = .524;
  public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.trackWidth);
  public static class DriverConstants {
    public static final int kDriverControllerPort = 0;

    public static final double kDriveP = 0.5;

    public static final double kTreadLength = 5 * Math.PI; 
  }
  
  public static class AutonConstants{
    public static final double kS = 0.21842;
    public static final double kV = 0.010489;
    public static final double kA = 0.0043684;
    public static final double kP = 0.0115;
  }

  public static class ManipulatorConstants {
    public static final int kManipulatorControllerPort = 1;
    public static final double kManipulatorControllerDeadZone = 0.1;

    public static final double armMaxSpeed = 0.3;
    public static final double manipulatorSpeed = 0.75;

    public static final int armLeaderPort = 21;
    public static final int armFollower1Port = 22;
    public static final int armFollower2Port = 23;
    public static final int manipulatorPort = 24;
    public static final int intakeLimitPort = 1;

    public static final int kClaw1Solenoid = 0;
    public static final int kClaw2Solenoid = 1;
    public static final int kClaw3Solenoid = 2;
    public static final int kClaw4Solenoid = 3;
    
    public static final int armEncoderPort = 0;

    public static final double maxVoltage = 4.0;
    public static final double cycleTime = 50.0;
    public static final double kArmP = 0.25;
    public static final double kArmI = 0;
    public static final double kArmD = 850.0;

    public static final double maxSetPoint = 0.4d;
  }
}
