// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.networktables.*;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.lang.Math;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PhotonSimulation extends SubsystemBase {
    /** Creates a new PhotonSubsystem. */
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;

    double camDiagFOV = 170.0; // degrees - assume wide-angle camera
    double camPitch = 0.0; // degrees
    double camHeightOffGround = 0.7; // meters
    double maxLEDRange = 20; // meters
    int camResolutionWidth = 960; // pixels
    int camResolutionHeight = 720; // pixels
    double minTargetArea = 10; // square pixels

    SimVisionSystem simVision;

    public PhotonSimulation() {
        simVision = new SimVisionSystem(
        "5417camera",
        camDiagFOV,
        new Transform3d(
                new Translation3d(0, 0, camHeightOffGround), new Rotation3d(0, camPitch, 0)),
        maxLEDRange,
        camResolutionWidth,
        camResolutionHeight,
        minTargetArea);

        double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
        double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
        double tgtXPos = Units.feetToMeters(54);
        double tgtYPos =
                Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
        Pose3d farTargetPose =
                new Pose3d(
                        new Translation3d(tgtXPos, tgtYPos, targetHeight),
                        new Rotation3d(0.0, 0.0, 0.0));

        simVision.addSimVisionTarget(new SimVisionTarget(farTargetPose, targetWidth, targetHeight, -1));
    }

    //change this code to fit our robot
    public update() {
        double leftMotorCmd = leftLeader.getSpeed();
        double rightMotorCmd = rightLeader.getSpeed();

        if (DriverStation.isEnabled() && !RobotController.isBrownedOut()) {
            leftMotorCmd = leftLeader.getSpeed();
            rightMotorCmd = rightLeader.getSpeed();
        }

        drivetrainSimulator.setInputs(
                leftMotorCmd * RobotController.getInputVoltage(),
                -rightMotorCmd * RobotController.getInputVoltage());
        drivetrainSimulator.update(0.02);

        // Update PhotonVision based on our new robot position.
        simVision.processFrame(drivetrainSimulator.getPose());

        field.setRobotPose(drivetrainSimulator.getPose());
    }
     
}
