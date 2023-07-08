// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.RameseteCommands;
import frc.robot.Constants;

import java.util.List;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class BackwardEngageAutoConeScore2 extends CommandBase {
  RamseteCommand ramseteCommand1;
  Drive drive;
  Trajectory translatedMoveBack;

  public BackwardEngageAutoConeScore2(Drive drive) {
    this.drive = drive;
    
    SimpleMotorFeedforward motorFF = 
      new SimpleMotorFeedforward(
        Constants.AutonConstants.kS,
        Constants.AutonConstants.kV, 
        Constants.AutonConstants.kA);

    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        motorFF, 
        Constants.kinematics, 10);

    TrajectoryConfig config = 
      new TrajectoryConfig(Constants.AutonConstants.chargeMaxSpeed2, Constants.AutonConstants.chargeMaxAcceleration)
          .setKinematics(Constants.kinematics).addConstraint(autoVoltageConstraint).setReversed(true);
 
    Trajectory moveBack = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,new Rotation2d(0)), 
      List.of(
        new Translation2d(-0.1,0)
      ), 
      new Pose2d(-0.2,0, new Rotation2d(0)),
      config);
    
    RamseteController ramseteControl1 = new RamseteController();
    
    ramseteCommand1 = new RamseteCommand(
      moveBack, 
      drive::getPose,
      ramseteControl1, 
      motorFF,
      Constants.kinematics, 
      drive::getWheelSpeeds, 
      new PIDController(Constants.AutonConstants.kP, Constants.AutonConstants.kI, Constants.AutonConstants.kD), 
      new PIDController(Constants.AutonConstants.kP, Constants.AutonConstants.kI, Constants.AutonConstants.kD),
      drive::setDriveVolts, 
      drive);
  }

  public Command getRamseteCommand (){
     return ramseteCommand1.andThen(() -> drive.SetSpeed(0.0, 0.0));
  }
}