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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final PhotonSubsystem m_photonsubsystem = new PhotonSubsystem();
  private final PhotonCommand m_pPhotonCommand = new PhotonCommand(m_photonsubsystem);

  public static final AHRS ahrs = new AHRS(SerialPort.Port.kMXP); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */
  private final NavXGyro m_NavXGyro = new NavXGyro();
  private final NavXGyroCommand m_NavXGyroCommand = new NavXGyroCommand(m_NavXGyro, ahrs);

  private static final double kP = 2;
  private static final double kI = 0;
  private static final double kD = 0;

  private static final PIDController pid = new PIDController(kP, kI, kD);

  private final Drive drive;
  public TankDrive(Drive subsystem) {
    drive = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ahrs.reset();
    ahrs.calibrate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setPower(RobotContainer.getDriverLeftJoystick(), RobotContainer.getDriverRightJoystick());
    if(RobotContainer.getButtonA()) {
      CommandScheduler.getInstance().schedule(m_pPhotonCommand);
      CommandScheduler.getInstance().schedule(m_NavXGyroCommand);
    } else {
      m_pPhotonCommand.cancel();
      m_NavXGyroCommand.cancel();
    }
    if(RobotContainer.getButtonX()) {
      ahrs.reset();
      ahrs.calibrate();
      final double angle_snapshot = m_NavXGyro.getGyroAngle(ahrs);
      pid.setSetpoint(90);
      pid.setTolerance(1, 10);
      
      while (!pid.atSetpoint()) {
        double leftPower = -MathUtil.clamp(pid.calculate(m_NavXGyro.getGyroAngle(ahrs)), -0.3, 0.3);
        double rightPower = MathUtil.clamp(pid.calculate(m_NavXGyro.getGyroAngle(ahrs)), -0.3, 0.3);
        drive.setPower(leftPower, rightPower);
      }
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
