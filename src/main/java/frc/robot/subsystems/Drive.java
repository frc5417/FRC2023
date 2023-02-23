// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  private static double LeftDistance = 0;
  private static double rightDistance = 0;

  private final static AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

  private final static CANSparkMax leftLeader = new CANSparkMax(Constants.DriveLeftLeader, MotorType.kBrushless);
  private final static CANSparkMax leftFollower = new CANSparkMax(Constants.DriveLeftFollower, MotorType.kBrushless);
  private final static CANSparkMax rightLeader = new CANSparkMax(Constants.DriveRightLeader, MotorType.kBrushless);
  private final static CANSparkMax rightFollower = new CANSparkMax(Constants.DriveRightFollower, MotorType.kBrushless);

  private final static MotorControllerGroup leftMotors = new MotorControllerGroup(leftLeader, leftFollower);
  private final static MotorControllerGroup rightMotors = new MotorControllerGroup(rightLeader, rightFollower);

  //private final static DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  private final static RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final static RelativeEncoder rightEncoder = rightLeader.getEncoder();

  private static DifferentialDriveOdometry odometry;

  private final static DoubleSolenoid ShifterL = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
  private final static DoubleSolenoid ShifterR = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 4);
  /** Creates a new Drive. */

  public Drive() {
    
    rightMotors.setInverted(false);
    leftMotors.setInverted(true);
/*
    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);
*/

    leftEncoder.setPositionConversionFactor(Constants.DriverConstants.kTreadLength);
    rightEncoder.setPositionConversionFactor(Constants.DriverConstants.kTreadLength);

    resetEncoders();

    odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    ShifterL.set(DoubleSolenoid.Value.kReverse);
    ShifterR.set(DoubleSolenoid.Value.kReverse);
    ahrs.calibrate();


    System.out.println(ahrs.getPitch());
    
  }

  public double GyroPitch(){
    return ahrs.getPitch();
  }
  
  public double GyroRoll(){
    return ahrs.getRoll();
  }

  public void SetSpeed(double leftSpeed, double rightSpeed) {
    leftMotors.set(leftSpeed);
    rightMotors.set(rightSpeed);
  }

  public void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void shiftToggle() {
    ShifterL.set(ShifterL.get() == DoubleSolenoid.Value.kReverse ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    ShifterR.set(ShifterR.get() == DoubleSolenoid.Value.kReverse ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  public double getPowerFromTilt(double tilt){
    return Math.abs(tilt/90);
  }


  public boolean balance(){

    if(GyroRoll() >= Constants.degreesAllowed && !ahrs.isCalibrating()){
      SetSpeed(clamp(-getPowerFromTilt(GyroRoll()), -.3, .3), clamp(-getPowerFromTilt(GyroRoll()), -.3, .3));
    }
    System.out.println(GyroRoll());

    if(GyroRoll() <= -(Constants.degreesAllowed) && !ahrs.isCalibrating()){
      SetSpeed(clamp(getPowerFromTilt(GyroRoll()), -.3, .3), clamp(getPowerFromTilt(GyroRoll()), -.3, .3));
    } 
    
    if(Math.abs(GyroRoll()) <= Constants.degreesAllowed && !ahrs.isCalibrating()){
      return true;
    } 
    return false;
  }

  @Override
  public void periodic() {
    odometry.update(ahrs.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(),rightEncoder.getVelocity());
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(ahrs.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
  }

  public void zeroHeading(){
    ahrs.reset();
  }

  public double getHeading(){
    return ahrs.getRotation2d().getDegrees();
  }
}
