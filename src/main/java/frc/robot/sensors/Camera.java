// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {

  private static Camera instance = null;

  /**
   * Represents a distance measurement obtained from a camera sensor.
   */
  public class AprilTagMeasurement {
    public final double Xdistance;
    public final double Ydistance;
    public final double yaw;
    public final double ambiguity;
    public final int id;

    /**
     * Constructs a new DistMeasurement object with the specified distance and
     * ambiguity values.
     * 
     * @param Xdistance the measured distance in the X axis
     * @param Ydistance the measured distance in the Y axis
     * @param yaw       the measured yaw
     * @param ambiguity the ambiguity of the distance measurement
     * @param id        the fiducial id of the detected apriltag
     */
    public AprilTagMeasurement(double Xdistance, double Ydistance, double yaw, double ambiguity, int id) {
      this.Xdistance = Xdistance;
      this.Ydistance = Ydistance;
      this.yaw = yaw;
      this.ambiguity = ambiguity;
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
}