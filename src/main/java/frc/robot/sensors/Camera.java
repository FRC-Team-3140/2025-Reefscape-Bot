// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.FieldAprilTags;
import frc.robot.subsystems.odometry.Odometry;

public class Camera extends SubsystemBase {
  private static Camera instance = null;

  private final double delayTime = 5;

  private boolean connected = false;

  private double lastIteration = 0;

  private Pose2d lastPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();

  private PhotonCamera front = new PhotonCamera("front");
  private PhotonCamera back = new PhotonCamera("back");

  private Transform3d frontToBot = new Transform3d(0, Constants.CameraConstants.frontOffsetToCenter, 0,
      new Rotation3d(0, (5 * Math.PI / 36), 0));
  private Transform3d backToBot = new Transform3d(0, Constants.CameraConstants.backOffsetToCenter,
      Constants.CameraConstants.backOffsetToCenterVert, new Rotation3d(0, 0, Math.PI));

  private AprilTagFieldLayout layout = FieldAprilTags.getInstance().field;
  private PhotonPoseEstimator frontEstimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS,
      frontToBot);
  private PhotonPoseEstimator backEstimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS,
      backToBot);

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

      // System.out.println("Connected: " + connected + " Front: " +
      // front.isConnected() + " Back: " + back.isConnected());

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
      Pose2d curPose = Odometry.getInstance().getPose();

      PhotonPipelineResult frontResult = front.getLatestResult();
      PhotonPipelineResult backResult = back.getLatestResult();

      if (frontResult.hasTargets() && backResult.hasTargets()) {
        frontEstimator.setReferencePose(curPose);
        backEstimator.setReferencePose(curPose);

        Optional<EstimatedRobotPose> frontPoseOpt = frontResult.getBestTarget().getPoseAmbiguity() < 0.2 ? frontEstimator.update(frontResult) : null;
        Optional<EstimatedRobotPose> backPoseOpt = backResult.getBestTarget().getPoseAmbiguity() < 0.2 ?  backEstimator.update(backResult) : null;

        if (frontPoseOpt.isPresent() && backPoseOpt.isPresent()) {
          Pose2d frontPoseEstimation = frontPoseOpt.get().estimatedPose.toPose2d();
          Pose2d backPoseEstimation = backPoseOpt.get().estimatedPose.toPose2d();

          double avgX = (frontPoseEstimation.getX() + backPoseEstimation.getX()) / 2;
          double avgY = (frontPoseEstimation.getY() + backPoseEstimation.getY()) / 2;
          double avgRot = (frontPoseEstimation.getRotation().getRadians()
              + backPoseEstimation.getRotation().getRadians()) / 2;

          estimatedPose = new Pose2d(avgX, avgY, new Rotation2d(avgRot));

          if (estimatedPose.getX() != lastPose.getX()) {
            lastPose = estimatedPose;
            return estimatedPose;
          } else {
            return null;
          }
        } else {
          return null;
        }
      } else if (frontResult.hasTargets()) {
        frontEstimator.setReferencePose(curPose);

        Optional<EstimatedRobotPose> frontPoseOpt = frontResult.getBestTarget().getPoseAmbiguity() < 0.2 ? frontEstimator.update(frontResult) : null;

        if (frontPoseOpt.isPresent()) {
          estimatedPose = frontPoseOpt.get().estimatedPose.toPose2d();
          if (estimatedPose.getX() != lastPose.getX()) {
            lastPose = estimatedPose;
            return estimatedPose;
          } else {
            return null;
          }
        }
      } else if (backResult.hasTargets()) {
        backEstimator.setReferencePose(curPose);
        
        Optional<EstimatedRobotPose> backPoseOpt = backResult.getBestTarget().getPoseAmbiguity() < 0.2 ?  backEstimator.update(backResult) : null;

        if (backPoseOpt.isPresent()) {
          estimatedPose = backPoseOpt.get().estimatedPose.toPose2d();
          if (estimatedPose.getX() != lastPose.getX()) {
            lastPose = estimatedPose;
            return estimatedPose;
          } else {
            return null;
          }
        }
      } else {
        return null;
      }
    } else {
      return null;
    }
    return null;
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