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

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;


/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final PhotonSubsystem m_photonsubsystem = new PhotonSubsystem();
  private final PhotonCommand m_pPhotonCommand = new PhotonCommand(m_photonsubsystem);

  private final Drive drive;

  public static final AHRS ahrs = new AHRS(SerialPort.Port.kMXP); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */
  private final NavXGyro m_NavXGyro = new NavXGyro();
  private final NavXGyroCommand m_NavXGyroCommand;



  
  public TankDrive(Drive subsystem) {
    drive = subsystem;

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
    // m_NavXGyroCommand.setAngle(180);
    // CommandScheduler.getInstance().schedule(m_NavXGyroCommand);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setPower(RobotContainer.getDriverLeftJoystick(), RobotContainer.getDriverRightJoystick());
    if(RobotContainer.getButtonA()) {
      CommandScheduler.getInstance().schedule(m_pPhotonCommand);
    } else {
      m_pPhotonCommand.cancel();
    }
    if(RobotContainer.getButtonX()) {
      System.out.println("Button Press Detected");
      m_NavXGyroCommand.setAngle(180);
      
      //   try {
      //     Thread.sleep(2000);
      //   } catch (Exception e) {
      //     System.out.println(e);
      //   }
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
