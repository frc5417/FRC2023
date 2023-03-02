// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


import com.kauailabs.navx.frc.AHRS;


/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final PhotonCommand m_pPhotonCommand;

  private final Drive drive;

  public static AHRS ahrs; /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */
  private final NavXGyroCommand m_NavXGyroCommand;
  
  public TankDrive(Drive subsystem, AHRS ahrs_passed, PhotonCommand photon_command_passed, NavXGyroCommand gyro_command_passed) {
    drive = subsystem;

    ahrs = ahrs_passed;

    m_pPhotonCommand = photon_command_passed;
    m_NavXGyroCommand = gyro_command_passed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
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
      double angle = m_pPhotonCommand.getYawFromSubsystem() * -1;
      System.out.println(angle);
      if (angle < 0) {
        angle -= 90;
      } else if (angle > 0) {
        angle += 90;
      }
      
      m_NavXGyroCommand.setAngle(angle);
    } else {
      m_NavXGyroCommand.setAngle(0);
      drive.setPower(0, 0);
    }
    if (Math.abs(RobotContainer.getDriverLeftJoystick()) > 0.1 || Math.abs(RobotContainer.getDriverRightJoystick()) > 0.1) {
      m_NavXGyroCommand.cancel();
      // drive.setPower(RobotContainer.getDriverLeftJoystick(), RobotContainer.getDriverRightJoystick());
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