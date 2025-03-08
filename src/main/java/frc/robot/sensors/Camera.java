// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.NetworkTables;
import frc.robot.libs.Vector2;

public class Camera extends SubsystemBase {
  private static Camera instance = null;

  private final double delayTime = 0.5;

  private boolean connected = false;

  private double globalTime = -1;
  private double lastTime = -1;
  private double frameTime = -1;

  private double lastAttemptReconnectIterration = 0;

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
    if (((Timer.getFPGATimestamp() - lastAttemptReconnectIterration)) > delayTime) {
      lastTime = globalTime;
      globalTime = NetworkTables.globalCameraTimestamp.getDouble(-1);
      frameTime = NetworkTables.camera0_Timestamp.getDouble(-1);

      if (globalTime == -1 || globalTime - frameTime > Constants.CameraConstants.maxTimeBeteweenFrames
          || globalTime == lastTime) {
        connected = false;
      } else {
        connected = true;
      }

      lastAttemptReconnectIterration = Timer.getFPGATimestamp();
    }

    Pose2d camPose = getPoseFromCamera();
    if (camPose != null)
      NetworkTables.cameraPose
          .setDoubleArray(new double[] {
              camPose.getX(),
              camPose.getY(),
              camPose.getRotation().getDegrees()
          });
  }

  public boolean isConnected() {
    return connected;
  }

  public Integer getClosestApriltag() {
    if (connected) {
      double[] distances = NetworkTables.camera0_Distances.getDoubleArray(new double[0]);
      int[] ids = getDetectedTags();

      if (distances.length <= 0 || ids.length <= 0)
        return null;

      double minDistance = distances[0];

      int minIndex = 0;

      for (int i = 0; i < distances.length; i++) {
        if (distances[i] < minDistance) {
          minDistance = distances[i];
          minIndex = i;
        }
      }

      return ids[minIndex];
    } else {
      return null;
    }
  }

  public AprilTagMeasurement getMeasurement(int id) {
    if (connected) {
      NetworkTables.camera0_requestedID.setDouble(id);

      Timer.delay(0.1);

      double globalCameraTimestamp = NetworkTables.camera0_Timestamp.getDouble(0);
      double timestamp = NetworkTables.camera0_requestedTimestamp.getDouble(-1);

      if (timestamp == -1 || globalCameraTimestamp - timestamp > Constants.CameraConstants.maxTimeBeteweenFrames)
        return null;

      return new AprilTagMeasurement(
          NetworkTables.camera0_requestedDistance.getDouble(0),
          NetworkTables.camera0_requestedBearing.getDouble(0),
          id);
    } else {
      return null;
    }
  }

  public Pose2d getPoseFromCamera() {
    if (connected) {
      double[] pose = NetworkTables.camera0_Position.getDoubleArray(new double[0]);
      double[] dir = NetworkTables.camera0_Direction.getDoubleArray(new double[0]);

      if (pose.length == 0 || dir.length == 0)
        return null;

      Vector2 poseVec = new Vector2(pose[0], pose[1]);
      Vector2 dirVec = new Vector2(dir[0], dir[1]);

      Vector2 oneMUnitVec = dirVec.sub(poseVec);
      Vector2 centerOfBot = oneMUnitVec.neg().mult(Constants.CameraConstants.aprilOffsetToCenter).add(poseVec);

      return new Pose2d(centerOfBot.X, centerOfBot.Y,
          new Rotation2d(Math.atan2(oneMUnitVec.Y, oneMUnitVec.X)));
    } else {
      return null;
    }
  }

  public int[] getDetectedTags() {
    if (connected) {
      double[] detectedTags = NetworkTables.camera0_IDs.getDoubleArray(new double[0]);

      if (detectedTags.length == 0)
        return null;

      int[] detectedTagsInt = new int[detectedTags.length];

      for (int i = 0; i < detectedTags.length; i++) {
        detectedTagsInt[i] = (int) detectedTags[i];
      }

      return detectedTagsInt;
    } else {
      return null;
    }
  }
}