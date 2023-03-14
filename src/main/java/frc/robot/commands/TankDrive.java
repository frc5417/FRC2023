// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.kauailabs.navx.frc.AHRS;
import org.photonvision.PhotonCamera;


/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private NavXGyroCommand m_NavXGyroCommand;

  private final Drive drive;
  private final PhotonSubsystem pcw;

  public AHRS ahrs; /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */

  private int counter = 0;
  
  public TankDrive(Drive drivetrain, PhotonSubsystem photon, AHRS ahrs_passed, NavXGyroCommand gyro_command_passed) {
    drive = drivetrain;
    pcw = photon;

    ahrs = ahrs_passed;
    m_NavXGyroCommand = gyro_command_passed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ahrs.reset();
    ahrs.calibrate();
    //m_NavXGyroCommand.setAngle(0);
    //CommandScheduler.getInstance().schedule(m_NavXGyroCommand);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter++ % 10 == 0) { pcw.updatePose(); }

    if (Math.abs(RobotContainer.getDriverLeftJoystick()) > 0.1 || Math.abs(RobotContainer.getDriverRightJoystick()) > 0.1) {
      drive.setPower(RobotContainer.getDriverLeftJoystick(), RobotContainer.getDriverRightJoystick());
    } else {
      drive.setPower(0, 0);
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