// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  private double LeftDistance = 0;
  private double rightDistance = 0;

  private final AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

  private final CANSparkMax leftLeader = new CANSparkMax(Constants.DriveLeftLeader, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(Constants.DriveLeftFollower, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(Constants.DriveRightLeader, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(Constants.DriveRightFollower, MotorType.kBrushless);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftLeader, leftFollower);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightLeader, rightFollower);

  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

  private final DifferentialDriveOdometry odometry;

  Solenoid ShifterL = new Solenoid(PneumaticsModuleType.REVPH, 0);
  Solenoid ShifterR = new Solenoid(PneumaticsModuleType.REVPH, 1);
  /** Creates a new Drive. */
  public Drive() {
    rightMotors.setInverted(true);
    leftMotors.setInverted(false);

    leftEncoder.setPositionConversionFactor(Constants.DriverConstants.kTreadLength);
    rightEncoder.setPositionConversionFactor(Constants.DriverConstants.kTreadLength);

    resetEncoders();

    odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  public double GyroPitch(){
    return ahrs.getPitch();
  }

  public void SetSpeed(double leftSpeed, double rightSpeed) {
    leftMotors.set(leftSpeed);
    rightMotors.set(rightSpeed);
  }

  public void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void ShiftToggle() {
    ShifterL.set(!ShifterL.get());
    ShifterR.set(!ShifterR.get());
  }

  public void balance(){
    while(GyroPitch() >= Constants.degreesAllowed){
      SetSpeed(-0.1, -0.1);
    }

    while(GyroPitch() <= -(Constants.degreesAllowed)){
      SetSpeed(0.1, 0.1);
    }
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
