// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Camera extends SubsystemBase {
  private static Camera instance = null;

  private final double delayTime = 0.5;

  private boolean connected = false;

  private double lastIteration = 0;

  private Pose2d lastPose = new Pose2d();

  private PhotonCamera front = new PhotonCamera("front");
  private PhotonCamera back = new PhotonCamera("back");

  /**
   * Represents a distance measurement obtained from a camera sensor.
   */
  public class AprilTagMeasurement {
    public final double distance;
    public final double yaw;
    public final int id;

    /**
     * Constructs a new DistMeasurement object with the specified distance and
     * ambiguity values.
     * 
     * @param Xdistance the measured distance in the X axis
     * @param Ydistance the measured distance in the Y axis
     * @param yaw       the measured yaw
     * @param id        the fiducial id of the detected apriltag
     */
    public AprilTagMeasurement(double distance, double yaw, int id) {
      this.distance = distance;
      this.yaw = yaw;
      this.id = id;
    }
  }

  /**
   * Represents a camera used for vision processing.
   * 
   * @return instance of Camera class
   */
  public static Camera getInstance() {
    if (instance == null) {
      instance = new Camera();
    }
    return instance;
  }

  /**
   * Represents a camera used for vision processing.
   * This class handles the connection and configuration of the camera.
   *
   * @param PhotonvisionConnectionAttempts The number of connection attempts to
   *                                       make to PhotonVision.
   * @param delayBetweenAttempts           The delay in seconds between each
   *                                       connection attempt.
   * @param minAmbiguity                   The minimum ambiguity value for
   *                                       AprilTags.
   */
  private Camera() {

  }

  @Override
  public void periodic() {
    if (((Timer.getFPGATimestamp() - lastIteration)) > delayTime) {
      isConnected();


      lastIteration = Timer.getFPGATimestamp();
    }
  }

  public boolean isConnected() {
    connected = (front.isConnected() && back.isConnected());
    return connected;
  }

  public Integer getClosestApriltag() {
    if (connected) {
      PhotonTrackedTarget frontTarget = front.getLatestResult().getBestTarget();
      PhotonTrackedTarget backTarget = back.getLatestResult().getBestTarget();

      if (frontTarget == null && backTarget == null) {
        return null;
      } else if (frontTarget == null) {
        return backTarget.getFiducialId();
      } else if (backTarget == null) {
        return frontTarget.getFiducialId();
      } else {
        double frontDistance = frontTarget.getBestCameraToTarget().getTranslation().getNorm();
        double backDistance = backTarget.getBestCameraToTarget().getTranslation().getNorm();
        return frontDistance < backDistance ? frontTarget.getFiducialId() : backTarget.getFiducialId();
      }
    } else {
      return null;
    }
  }

  public Pose2d getPoseFromCamera() {
    if (connected) {
      Optional<MultiTargetPNPResult> frontResult = front.getLatestResult().multitagResult;
      Optional<MultiTargetPNPResult> backResult = back.getLatestResult().multitagResult;
      Pose2d robotPose = new Pose2d();

      if (frontResult.isPresent() && backResult.isPresent()) {
       
        if (frontResult.get().estimatedPose.ambiguity > Constants.CameraConstants.minAmbiguity
            || backResult.get().estimatedPose.ambiguity > Constants.CameraConstants.minAmbiguity) {
          return null;
        }
        Transform3d poseToFront = frontResult.get().estimatedPose.best;
        Transform3d poseToBack = backResult.get().estimatedPose.best;

        // Combine the poses from the front and back cameras to calculate the robot's
        // pose
        robotPose = new Pose2d(
            (poseToFront.getTranslation().getX() + poseToBack.getTranslation().getX()) / 2,
            (poseToFront.getTranslation().getY() + poseToBack.getTranslation().getY()) / 2,
            new Rotation2d((poseToFront.getRotation().getZ() + poseToBack.getRotation().getZ()) / 2));

        
      } else if (frontResult.isPresent()){

        if (frontResult.get().estimatedPose.ambiguity > Constants.CameraConstants.minAmbiguity) {
          return null;
        }
        Transform3d poseToFront = frontResult.get().estimatedPose.best;

        robotPose = new Pose2d(
            poseToFront.getTranslation().getX(),
            poseToFront.getTranslation().getY(),
            new Rotation2d(poseToFront.getRotation().getZ()));
        
        

      } else if (backResult.isPresent()){

        if (backResult.get().estimatedPose.ambiguity > Constants.CameraConstants.minAmbiguity) {
          return null;
        }
        Transform3d poseToBack = backResult.get().estimatedPose.best;

        robotPose = new Pose2d(
            poseToBack.getTranslation().getX(),
            poseToBack.getTranslation().getY(),
            new Rotation2d(poseToBack.getRotation().getZ()));

      } else {
        return null;
      }
      if(robotPose.getX() != lastPose.getX()) { 
        lastPose = robotPose;
        return  robotPose ;
      } else {
        lastPose = robotPose;
        return  robotPose ;
      }
    } else {
      return null;
    }
  }

  public int[] getDetectedTags() {
    if (connected) {
      List<PhotonTrackedTarget> detectedTags = front.getLatestResult().targets;
      List<PhotonTrackedTarget> detectedTagsBack = back.getLatestResult().targets;

      if (detectedTags.size() == 0)
        return null;

      int[] detectedTagsInt = new int[detectedTags.size() + detectedTagsBack.size()];

      for (int i = 0; i < detectedTags.size(); i++) {
        detectedTagsInt[i] = (int) detectedTags.get(i).getFiducialId();
      }

      for (int i = 0; i < detectedTagsBack.size(); i++) {
        detectedTagsInt[i + detectedTags.size()] = (int) detectedTagsBack.get(i).getFiducialId();
      }

      return detectedTagsInt;
    } else {
      return null;
    }
  }
}