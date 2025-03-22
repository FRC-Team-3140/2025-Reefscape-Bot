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

  private Pose2d lastPose = new Pose2d();
  private Vector2 lastVecFront = new Vector2();
  private Vector2 lastVecBack = new Vector2();

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

    // Pose2d camPose = getPoseFromCamera();
    // if (camPose != null)
    // NetworkTables.cameraPose
    // .setDoubleArray(new double[] {
    // camPose.getX(),
    // camPose.getY(),
    // camPose.getRotation().getDegrees()
    // });
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
      boolean camera0Exists = false;
      boolean camera2Exists = false;
      // Camera 0 Calculation
      double[] pose0 = NetworkTables.camera0_Position.getDoubleArray(new double[0]);
      double[] dir0 = NetworkTables.camera0_Direction.getDoubleArray(new double[0]);
      Vector2 poseVec0 = null, dirVec0 = null, mUnitVec0 = null, centerOfBot0 = null;

      if (pose0.length != 0 && dir0.length != 0) {
        poseVec0 = new Vector2(pose0[0], pose0[1]);
        dirVec0 = new Vector2(dir0[0], dir0[1]);

        mUnitVec0 = dirVec0.sub(poseVec0);
        centerOfBot0 = mUnitVec0.neg().mult(Constants.CameraConstants.aprilOffsetToCenter0).add(poseVec0);

        camera0Exists = true;
      }

      // Camera 2 Calculation
      double[] pose2 = NetworkTables.camera2_Position.getDoubleArray(new double[0]);
      double[] dir2 = NetworkTables.camera2_Direction.getDoubleArray(new double[0]);
      Vector2 poseVec2 = null, dirVec2 = null, mUnitVec2 = null, centerOfBot2 = null;
      if (pose2.length != 0 && dir0.length != 0) {
        poseVec2 = new Vector2(pose2[0], pose2[1]);
        dirVec2 = new Vector2(dir2[0], dir2[1]);

        mUnitVec2 = dirVec2.sub(poseVec2);
        centerOfBot2 = mUnitVec2.neg().mult(Constants.CameraConstants.aprilOffsetToCenter2).add(poseVec2);

        camera2Exists = true;
      }

      if (camera0Exists && poseVec0.X == lastVecFront.X)
        camera0Exists = false;

      if (camera0Exists)
        lastVecFront = poseVec0;

      if (camera2Exists && poseVec2.X == lastVecBack.X)
        camera2Exists = false;

      if (camera2Exists)
        lastVecBack = poseVec2;

      Pose2d curPose = null;

      if (camera0Exists && camera2Exists) {
        curPose = new Pose2d((centerOfBot0.X + centerOfBot2.X) / 2, (centerOfBot0.Y + centerOfBot2.Y) / 2,
            new Rotation2d(
                (Math.atan2(mUnitVec0.Y, mUnitVec0.X) + Math.atan2(mUnitVec2.neg().Y, mUnitVec2.neg().X)) / 2));
      } else if (camera0Exists) {
        curPose = new Pose2d(centerOfBot0.X, centerOfBot0.Y,
            new Rotation2d(Math.atan2(mUnitVec0.Y, mUnitVec0.X)));
      } else if (camera2Exists) {
        curPose = new Pose2d(centerOfBot2.X, centerOfBot2.Y,
            new Rotation2d(Math.atan2(mUnitVec2.neg().Y, mUnitVec2.neg().X)));
      } else {
        return null;
      }

      if (curPose.getX() == lastPose.getX())
        return null;

      if (camera0Exists) {
        NetworkTables.frontCameraPose
            .setDoubleArray(new double[] {
                centerOfBot0.X,
                centerOfBot0.Y,
                new Rotation2d(Math.atan2(mUnitVec0.Y, mUnitVec0.X)).getDegrees()
            });
      }

      if (camera2Exists) {
        NetworkTables.backCameraPose
            .setDoubleArray(new double[] {
                centerOfBot2.X,
                centerOfBot2.Y,
                new Rotation2d(Math.atan2(mUnitVec2.neg().Y, mUnitVec2.neg().X)).getDegrees()
            });
      }

      lastPose = curPose;
      return curPose.getX() != 0 ? curPose : null;
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