// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import com.pathplanner.lib.PathPlanner;

public class BackwardAutoConeScore3 extends CommandBase {
  RamseteCommand ramseteCommand1;
  Drive drive;
  Trajectory translatedMoveBack;

  public BackwardAutoConeScore3(Drive drive) {
     
    this.drive = drive;
    SimpleMotorFeedforward motorFF = new SimpleMotorFeedforward(Constants.AutonConstants.kS, Constants.AutonConstants.kV, Constants.AutonConstants.kA);

    Trajectory moveBack = PathPlanner.loadPath("reverseMove2", Constants.AutonConstants.autoMaxSpeed, Constants.AutonConstants.autoMaxAcceleration, false);
    
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
    return ramseteCommand1.andThen(() -> drive.SetSpeed(0, 0));
  }
}