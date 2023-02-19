// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;

import java.util.List;
import com.pathplanner.lib.PathPlanner;

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

public class AutoStack1 extends CommandBase {
  RamseteCommand ramseteCommand1;
  Drive drive;

  Trajectory moveBack;

  public AutoStack1(Drive drive) {
    this.drive = drive;
    SimpleMotorFeedforward motorFF = new SimpleMotorFeedforward(Constants.AutonConstants.kS, Constants.AutonConstants.kV, Constants.AutonConstants.kA);
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        motorFF, 
        Constants.kinematics, Constants.maxVoltage);

    TrajectoryConfig config = 
      new TrajectoryConfig(Constants.maxSpeed, Constants.maxAcceleration)
          .setKinematics(Constants.kinematics).addConstraint(autoVoltageConstraint);

    //first step is to move back slightly
    moveBack = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,new Rotation2d(0)), 
      List.of(
        new Translation2d(1,0)
      ), 
      new Pose2d(1,0, new Rotation2d(0)), 
      config);
    

    drive.resetOdometry(moveBack.getInitialPose());

    RamseteController ramseteControl1 = new RamseteController();

    ramseteCommand1 = new RamseteCommand(
      moveBack, 
      drive::getPose,
      ramseteControl1, 
      motorFF,
      Constants.kinematics, 
      drive::getWheelSpeeds, 
      new PIDController(Constants.AutonConstants.kP, 0, 0), 
      new PIDController(Constants.AutonConstants.kP, 0, 0), 
      drive::SetSpeed, 
      drive);
  }

  public Command getRamseteCommand (){
    return ramseteCommand1.andThen(() -> drive.SetSpeed(0, 0));
  }
}