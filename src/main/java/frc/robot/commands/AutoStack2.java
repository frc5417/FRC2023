// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class AutoStack2 extends CommandBase {
  RamseteCommand ramseteCommand1;
  Drive drive;
  Trajectory translatedMoveBack;

  public AutoStack2(Drive drive) {
    try {
      this.drive = drive;
      //reset odometry to be zero here
      //drive.resetOdometry(new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(0.0,0.0)));
    SimpleMotorFeedforward motorFF = new SimpleMotorFeedforward(Constants.AutonConstants.kS, Constants.AutonConstants.kV, Constants.AutonConstants.kA);
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        motorFF, 
        Constants.kinematics, 10);

    TrajectoryConfig config = 
      new TrajectoryConfig(Constants.AutonConstants.autoMaxSpeed, Constants.AutonConstants.autoMaxAcceleration)
          .setKinematics(Constants.kinematics).addConstraint(autoVoltageConstraint).setReversed(true);
    //first step is to move back slightly, old moveBack
    Trajectory moveBack = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,new Rotation2d(0)), 
      List.of(
        new Translation2d(-0.25,0),
        new Translation2d(-0.6,0),
        new Translation2d(-1.0,0)
      ), 
      new Pose2d(-1.5,0, new Rotation2d(0)), 
      config);
    
    
    //drive.resetOdometry(moveBack.getInitialPose());

    RamseteController ramseteControl1 = new RamseteController();

    //reset the pose:
    //Pose2d resetPose = new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(0.0,0.0));
    //drive.resetOdometry(resetPose);
    
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
    catch (Exception e) {
      //System.out.println("auto stack error: "+ e);
    }
    
  }

  public Command getRamseteCommand (){
    //return new StopAuton(drive);
    return ramseteCommand1.andThen(() -> drive.SetSpeed(0, 0));
  }
}