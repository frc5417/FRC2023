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
import org.photonvision.common.networktables.*;
import org.photonvision.targeting.PhotonTrackedTarget;



import java.lang.Math;

public class PhotonSubsystem extends SubsystemBase {
  /** Creates a new PhotonSubsystem. */
  public PhotonCamera photonCamera;
  public PhotonPoseEstimator photonPoseEstimator;

  private double[][] cameraInfo = new double[9][4]; // 1 indexed

  public PhotonSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public PhotonCamera PhotonCameraWrapper() {
    // Set up a test arena of two apriltags at the center of each driver station set
    final AprilTag tag18 =
            new AprilTag(
                    18,
                    new Pose3d(
                            new Pose2d(
                                    FieldConstants.length,
                                    FieldConstants.width / 2.0,
                                    Rotation2d.fromDegrees(180))));
    final AprilTag tag01 =
            new AprilTag(
                    01,
                    new Pose3d(new Pose2d(0.0, FieldConstants.width / 2.0, Rotation2d.fromDegrees(0.0))));
    ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
    atList.add(tag18);
    atList.add(tag01);

    // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
    AprilTagFieldLayout atfl =
            new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);

    // Forward Camera
    photonCamera =
            new PhotonCamera(
                    VisionConstants
                            .cameraName); // Change the name of your camera here to whatever it is in the
    // PhotonVision UI.

    // Create pose estimator
    photonPoseEstimator =
            new PhotonPoseEstimator(
                    atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, VisionConstants.robotToCam);
  // System.out.println(photonCamera.isConnected());
  return photonCamera;
}
  public double getYaw(PhotonCamera camera) {
    var result = camera.getLatestResult();
    if(result.hasTargets()) {
      return result.getBestTarget().getYaw();
    }
    else{
      return 0;
    }
  }

  public double getDistance(PhotonCamera camera) {
    var result = camera.getLatestResult();
    if(result.hasTargets()) {
      return PhotonUtils.calculateDistanceToTargetMeters(
              Constants.CAMERA_HEIGHT_METERS,
              Constants.TARGET_HEIGHT_METERS,
              Constants.CAMERA_PITCH_RADIANS,
              Math.toRadians(result.getBestTarget().getPitch()));
    }
    else{
      return 0;
    }
  }
    public void getPose(PhotonCamera camera) {
      var result = camera.getLatestResult();
      if(result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          cameraInfo[target.getFiducialId()][0] = target.getBestCameraToTarget().getX();
          cameraInfo[target.getFiducialId()][1] = target.getBestCameraToTarget().getY();
          cameraInfo[target.getFiducialId()][2] = target.getBestCameraToTarget().getZ();
          cameraInfo[target.getFiducialId()][3] = target.getYaw();

          System.out.print("FID: ");
          System.out.print(target.getFiducialId());
          System.out.print(", ");
          System.out.print("X: ");
          System.out.print(cameraInfo[target.getFiducialId()][0]);
          System.out.print(", ");
          System.out.print("Y: ");
          System.out.print(cameraInfo[target.getFiducialId()][1]);
          System.out.print(", ");
          System.out.print("Z: ");
          System.out.print(cameraInfo[target.getFiducialId()][2]);
          System.out.print(", ");
          System.out.print("Yaw: ");
          System.out.print(cameraInfo[target.getFiducialId()][3]);
          System.out.println("");
        }
        
        // System.out.println(result.getTargets().get(0).getrFiducialId());
      }
  }





  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
   *     of the observation. Assumes a planar field and the robot is always firmly on the ground
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      return photonPoseEstimator.update();
  }
}