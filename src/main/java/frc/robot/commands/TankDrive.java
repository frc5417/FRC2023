// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.PhotonSubsystem;


import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavXGyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.photonvision.PhotonCamera;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;


/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final PhotonSubsystem m_photonsubsystem;
  private final PhotonCommand m_pPhotonCommand;
  private final PhotonCamera camera;

  private final Drive drive;

  public static final AHRS ahrs = new AHRS(SerialPort.Port.kMXP); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */
  private final NavXGyro m_NavXGyro = new NavXGyro();
  private final NavXGyroCommand m_NavXGyroCommand;



  
  public TankDrive(Drive subsystem, PhotonSubsystem photonsub) {
    drive = subsystem;
    m_photonsubsystem = photonsub;

    camera = m_photonsubsystem.PhotonCameraWrapper();

    m_pPhotonCommand = new PhotonCommand(m_photonsubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    m_NavXGyroCommand = new NavXGyroCommand(m_NavXGyro, ahrs, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ahrs.reset();
    ahrs.calibrate();
    m_NavXGyroCommand.setAngle(0);
    CommandScheduler.getInstance().schedule(m_NavXGyroCommand);
    CommandScheduler.getInstance().schedule(m_pPhotonCommand);
    // m_NavXGyroCommand.setAngle(180);
    // CommandScheduler.getInstance().schedule(m_NavXGyroCommand);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.getButtonA()) {
      CommandScheduler.getInstance().schedule(m_pPhotonCommand);
    } else {
      m_pPhotonCommand.cancel();
    }

    if(RobotContainer.getButtonX()) { 
      m_NavXGyro.resetGyroAngle(ahrs);
      // System.out.println("Button Press Detected");
      // System.out.println(m_pPhotonCommand.getYawFromSubsystem());
      double angle =  m_photonsubsystem.getYaw(camera);
      System.out.println(angle);
      if (angle != 0) {
        m_NavXGyroCommand.setAngle(angle);
      } 
    } 
    if (Math.abs(RobotContainer.getDriverLeftJoystick()) > 0.1 || Math.abs(RobotContainer.getDriverRightJoystick()) > 0.1) {
      m_NavXGyroCommand.cancel();
      drive.setPower(RobotContainer.getDriverLeftJoystick(), RobotContainer.getDriverRightJoystick());
    } else {
      drive.setPower(0, 0);
      // m_NavXGyro.resetGyroAngle(ahrs);
      CommandScheduler.getInstance().schedule(m_NavXGyroCommand);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
