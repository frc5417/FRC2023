// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
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

  private final static DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  private final static RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final static RelativeEncoder rightEncoder = rightLeader.getEncoder();

  private static DifferentialDriveOdometry odometry;

  /** Creates a new Drive. */

  //initializes PID controller for balancing
  private final static PIDController balancePID = new PIDController(Constants.BalanceConstants.kP,Constants.BalanceConstants.kI,Constants.BalanceConstants.kD);
  private static int balanceCount = 0;

  //used for determining the acceleraton for controller rumble
  private static double lastSpeed = 0.0;
  private static double calculatedAcceleration = 0.0;
  private static int counter = 0;
  private static boolean firstRun = true;

  public Drive() {
    
    rightMotors.setInverted(false);
    leftMotors.setInverted(false);

    leftEncoder.setPositionConversionFactor(Constants.DriverConstants.kTreadLength);
    rightEncoder.setPositionConversionFactor(Constants.DriverConstants.kTreadLength);

    resetEncoders();

    odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    ahrs.calibrate();
    
    //set balance PID controller setpoint to 0 and have a max degrees allowed set in Constants
    balancePID.setSetpoint(0.0);
    balancePID.setTolerance(Constants.BalanceConstants.degreesAllowed);
  }
  public void setDriveBreak(){
    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);
  }

  public void setDriveCoast(){
    leftLeader.setIdleMode(IdleMode.kCoast);
    leftFollower.setIdleMode(IdleMode.kCoast);
    rightLeader.setIdleMode(IdleMode.kCoast);
    rightFollower.setIdleMode(IdleMode.kCoast);
  }

  public double GyroPitch(){
    return ahrs.getPitch();
  }
  
  public double GyroRoll(){
    return ahrs.getRoll();
  }

  public void SetSpeed(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
    drive.setSafetyEnabled(true);
    drive.feed();
  }

  public void setDriveVolts(double leftVolts, double rightVolts){
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  public void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  public double angleToVolts(double tilt) {
    double outputVolts = (tilt/90.0) * Constants.maxVoltage;
    if (Math.abs(outputVolts) >= Constants.BalanceConstants.balanceMaxVoltage) {
      double sign = outputVolts/Math.abs(outputVolts);
      outputVolts = sign*Constants.BalanceConstants.balanceMaxVoltage;
    }
    return outputVolts;
  }

  public void rumble(){
    double combinedSpeed = (getWheelSpeedsDouble()[0] + getWheelSpeedsDouble()[1]) / 2.0;
    //updates roughly every 3 times per second
    if (counter++ % 15 == 0) { 
      lastSpeed = combinedSpeed;
    }  
    if (firstRun) { 
      firstRun = false;
    }
    else {
      calculatedAcceleration = combinedSpeed - lastSpeed;
      double rumble = (calculatedAcceleration / Constants.maxAcceleration) * Constants.maxRumble;
      if (calculatedAcceleration < 0.2) {
        RobotContainer.setDriverRumble(0.0);
      } if (rumble > Constants.maxAcceleration) {
        rumble = Constants.maxRumble;
      }
      RobotContainer.setDriverRumble(rumble);
    }
  }

  public boolean pidBalance() {
    //if current gyro roll is positive then we want to set motor power forward to correct for the error
    double leftPower = angleToVolts(-balancePID.calculate(GyroRoll()));
    double rightPower = leftPower;

    if (balancePID.atSetpoint()) {
      //if less than degreesAllowed in Constants then stop moving
      balanceCount++;
      if (balanceCount == 25) {
        setDriveVolts(0.0, 0.0);
      }
      return true;
    } else {
      balanceCount = 0;
    }

    //if greater than degreesAllowed in Constants then move based on PID calculations
    setDriveVolts(leftPower, rightPower);
    return false;
  }

  @Override
  public void periodic() {
    drive.feed();
    odometry.update(ahrs.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    if(counter++ % 50 == 0){
      System.out.println(getPose());
    }
    
    rumble();    
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(),-rightEncoder.getVelocity());
  }

  public double[] getWheelSpeedsDouble() {
    double[] wheelSpeeds = {leftEncoder.getVelocity(), rightEncoder.getVelocity()};
    return wheelSpeeds;
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(ahrs.getRotation2d(), leftEncoder.getPosition(), -rightEncoder.getPosition(), pose);
  }

  public void zeroHeading(){
    ahrs.reset();
  }

  public double getHeading(){
    return ahrs.getRotation2d().getDegrees();
  }
}
