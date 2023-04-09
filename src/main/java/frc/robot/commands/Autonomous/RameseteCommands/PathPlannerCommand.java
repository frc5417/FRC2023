// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.RameseteCommands;
import frc.robot.Constants;
import com.pathplanner.lib.PathPlanner;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;

public class PathPlannerCommand extends CommandBase {
  RamseteCommand ramseteCommand;
  Drive drive;

  public PathPlannerCommand(Drive drive) {
    this.drive = drive;
    SimpleMotorFeedforward motorFF = new SimpleMotorFeedforward(Constants.AutonConstants.kS, Constants.AutonConstants.kV, Constants.AutonConstants.kA);

      //chargeb is for auton on blue side
        //chargeb1 drives from the right side starting point to the left spot on the charging station
        //chargeb2 is the safest and drives from the middle starting point on to the charging station
        //chargeb3 drives from the left side starting point to the right spot on the charging station
      //chargea is for auton on red side
        //charger1 drives from the right side starting point to the left spot on the charging station
        //charger2 is the safest and drives from the middle starting point on to the charging station
        //charger3 drives from the left side starting point to the right spot on the charging station
    Trajectory chargeStationOnly = PathPlanner.loadPath("chargeb1", Constants.maxSpeed, Constants.maxAcceleration);

    RamseteController ramseteControl = new RamseteController();

    ramseteCommand = new RamseteCommand(
      chargeStationOnly, 
      drive::getPose,
      ramseteControl, 
      motorFF,
      Constants.kinematics, 
      drive::getWheelSpeeds, 
      new PIDController(Constants.AutonConstants.kP, 0, 0), 
      new PIDController(Constants.AutonConstants.kP, 0, 0), 
      drive::SetSpeed, 
      drive);
  }

  public CommandBase getRamseteCommand (){
    return ramseteCommand.andThen(() -> drive.SetSpeed(0, 0));
  }
}