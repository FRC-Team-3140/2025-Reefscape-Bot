// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libs.NetworkTables;

public class Camera extends SubsystemBase {
  private static Camera instance = null;

  /**
   * Represents a distance measurement obtained from a camera sensor.
   */
  public class AprilTagMeasurement {
    public final double Xdistance;
    public final double Ydistance;
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
    public AprilTagMeasurement(double Xdistance, double Ydistance, double yaw, int id) {
      this.Xdistance = Xdistance;
      this.Ydistance = Ydistance;
      this.yaw = yaw;
      this.id = id;
    }
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

  public AprilTagMeasurement getClosestApriltag() {
    return new AprilTagMeasurement(
        0,
        0,
        0,
        0);
  }

  public AprilTagMeasurement getMeasurement(int id) {
    return new AprilTagMeasurement(
        0,
        0,
        0,
        id);
  }

  public Pose2d getCameraPose() {
    return new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(2)));
  }

  public int[] getDetectedTags() {
    double[] detectedTags = NetworkTables.ids.getDoubleArray(new double[0]);
    int[] detectedTagsInt = new int[detectedTags.length];

    for (int i = 0; i < detectedTags.length; i++) {
      detectedTagsInt[i] = (int) detectedTags[i];
    }

    return detectedTagsInt;
  }
}