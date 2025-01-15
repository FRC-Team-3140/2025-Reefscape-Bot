// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonVersion;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class Camera extends SubsystemBase {

  private static Camera instance = null;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private long programStartTime = System.currentTimeMillis();

  // Gets initial instantiation of Cameras - TK
  public PhotonCamera april = aprilGetInstance();
  public PhotonCamera sideApril = new PhotonCamera("Side April");
  public PhotonCamera notes = notesGetInstance();

  // Measurements to Cameras
  private Transform3d robotToApril = new Transform3d(new Translation3d(-Constants.Bot.botLength / 2, 0.0, .42),
      new Rotation3d(0, -Math.PI / 120, Math.PI));
  private Transform3d sideRobotToApril = new Transform3d(new Translation3d(0, -Constants.Bot.botLength / 2, 0.5),
      new Rotation3d(0, 0, -Math.PI / 2));

  // TODO: get measurements for note cam - TK : private Transform3d robotToNote =
  // new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private PhotonPoseEstimator aprilTagPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToApril);

  private PhotonPoseEstimator sideAprilTagPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, sideRobotToApril);

  private boolean connected = false;
  private int connectionAttempts = 2;

  private static boolean versionMatches = false;

  // This thread allows this connection check to run in the background and not
  // block other Subsystems.
  private Thread attemptReconnection = null;

  // Time to delay periodic method Networktable connection check. IN
  // Seconds!!
  private double delayTime = 5;

  // Time to delay connection attempts is in SECONDS! - TK
  private double attemptDelay;

  // TODO: Determine if we need the pathfind to apriltag code
  /*
   * // Global variables for updating Pathplanner poses - TK
   * private SwerveDrive swerveDrive;
   * private Pose2d currentSwervePose2d;
   * private double currentX;
   * private double currentY;
   * private double newX;
   * private double newY;
   *
   * 
   * // percentage of forward distance you want to drive before stopping (to
   * prevent
   * // crashing). - TK
   * private double percentTravelDist = 0.8; // Must be < 1
   */

  // Global fiducial ids for important landmark apriltags - TK
  public aprilTagLayout aprilTagLayout;
  private int speakerAprilTagC;
  private int ampAprilTag;
  private int sourceAprilTag;

  public class aprilTagLayout {
    public final int speakerAprilTag;
    public final int ampAprilTag;
    public final int sourceAprilTag;

    public aprilTagLayout(int speaker, int amp, int source) {
      this.speakerAprilTag = speaker;
      this.ampAprilTag = amp;
      this.sourceAprilTag = source;
    }
  }

  public class aprilTagLocation {
    public final boolean isDetected;
    public final double distance;
    public final double ambiguity;
    public final double angle;
    public final int id;

    public aprilTagLocation(boolean isDetected, double dist, double ambiguity, double angle, int id) {
      this.isDetected = isDetected;
      this.distance = dist;
      this.ambiguity = ambiguity;
      this.angle = angle;
      this.id = id;
    }
  }

  public class NoteLocation {
    public final boolean isDetected;
    public final double distance;
    public final double angle;

    public NoteLocation(boolean isDetected, double dist, double angle) {
      this.isDetected = isDetected;
      this.distance = dist;
      this.angle = angle;
    }
  }

  public class DistAmb {
    public final double distance;
    public final double ambiguity;

    public DistAmb(double distance, double ambiguity) {
      this.distance = distance;
      this.ambiguity = ambiguity;
    }
  }

  private Camera(SwerveDrive swerve, int PhotonvisionConnectionAttempts, double delayBetweenAttempts) {
    attemptDelay = delayBetweenAttempts;

    aprilTagPoseEstimator.setReferencePose(new Pose2d(0, 0, new Rotation2d()));

    while (!connected && connectionAttempts <= PhotonvisionConnectionAttempts) {
      if (inst.getTable("photonvision").getSubTables().contains("april")) {
        connected = true;
        versionMatches = checkVersion();
        aprilGetInstance();
        notesGetInstance();
        System.out.println("PhotonVision is connected and is probably working as expected...");
        break;
      } else {
        System.err.println("Photonvision Not Connected Properly!");
        connected = false;
        System.out.println("Attempt: " + connectionAttempts + "\nChecking for PhotonVision connection in "
            + attemptDelay + " seconds.");
        Timer.delay(attemptDelay);
        connectionAttempts++;
      }
    }

    setNetworktableStatus();

    // swerveDrive = swerve;

    // Will create a field layout object and set global variables for landmark
    // apriltags as mentioned earlier. This is not to be confused with the
    // Photonvision apriltag layout class! - TK
    configureTeam();
  }

  public static Camera getInstance() {
    if (instance == null) {
      instance = new Camera(SwerveDrive.getInstance(), 5, 1);
    }
    return instance;
  }

  // Attempt reconnection method attempts to reinstantiate cameras on
  // reconnection.
  // The "singleton" is just here to return the already created instance if there
  // is one - TK
  private PhotonCamera aprilGetInstance() {
    checkVersion();
    if (versionMatches && april == null) {
      april = new PhotonCamera(inst, "april");
    }
    return april;
  }

  // Attempt reconnection method attempts to reinstantiate cameras on
  // reconnection.
  // The "singleton" is just here to return the already created instance if there
  // is one - TK
  private PhotonCamera notesGetInstance() {
    checkVersion();
    if (versionMatches && notes == null) {
      notes = new PhotonCamera(inst, "notes");
    }
    return notes;
  }

  private void configureTeam() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      speakerAprilTagC = 4; // Apriltag on left
      ampAprilTag = 5;
      sourceAprilTag = 10; // Apriltag on left

      aprilTagLayout = new aprilTagLayout(speakerAprilTagC, ampAprilTag, sourceAprilTag);
    } else {
      speakerAprilTagC = 7; // Apriltag on left
      ampAprilTag = 6;
      sourceAprilTag = 2; // Apriltag on left

      aprilTagLayout = new aprilTagLayout(speakerAprilTagC, ampAprilTag, sourceAprilTag);
    }
  }

  private boolean checkVersion() {
    Timer timeout = new Timer();
    String version = "";

    timeout.start();

    while (connected && version == "" && !timeout.hasElapsed(delayTime)) {
      version = inst.getTable("photonvision").getEntry("version").getString("");
      if (version == "") {
        System.err.println("Photon version not available yet...");
      }
      if (timeout.hasElapsed(delayTime)) {
        System.err.println("checkVersion loop Timedout for safety!");
      }
    }

    return PhotonVersion.versionMatches(version);
  }

  private boolean testConnection() {
    // If either camera is working connected == true
    if (april.isConnected() || notes.isConnected()) {
      connected = true;
    } else {
      connected = false;
    }

    return connected;
  }

  private void attemptToReconnect() {
    System.err.println(
        "!!!!!!!!!!!!!!!!!!!!\nPhotonvision is no longer connected properly.\nAttempting reconnection\n!!!!!!!!!!!!!!!!!!!!");
    while (!connected) {
      if (testConnection()) {
        connected = true;
        versionMatches = checkVersion();
        aprilGetInstance();
        notesGetInstance();
        System.out.println("PhotonVision is connected and is probably working as expected...");
        break;
      } else {
        System.err.println("Photonvision Not Connected Properly!");
        connected = false;
        System.out.println("Checking for PhotonVision connection in " + attemptDelay + " seconds.");
        Timer.delay(attemptDelay);
      }
    }
  }

  private void setNetworktableStatus() {
    try {
      inst.getTable("Vision").getSubTable("Status").getEntry("Version Matches: ").setBoolean(versionMatches);
      inst.getTable("Vision").getSubTable("Status").getSubTable("Version Info").getEntry("Photon Version: ")
          .setString(inst.getTable("photonvision").getEntry("version").getString("Version not available..."));
      inst.getTable("Vision").getSubTable("Status").getSubTable("Version Info").getEntry("Photon Lib Version: ")
          .setString(PhotonVersion.versionString);
      inst.getTable("Vision").getSubTable("Status").getEntry("Connection: ").setBoolean(connected);

      if (connected && versionMatches && april != null) {
        inst.getTable("Vision").getSubTable("Status").getSubTable("Camera Status").getEntry("April Connection: ")
            .setBoolean(april.isConnected());
        inst.getTable("Vision").getSubTable("Status").getSubTable("Camera Status").getEntry("Notes Connection: ")
            .setBoolean(notes.isConnected());
      }
    } catch (Error e) {
      System.out.println("An error occured in Camera: \nUnable to publish status to Networktables:\n" + e);
    }
  }

  @Override
  public void periodic() {
    try {
      // Was using Timer.delay() function here, but this caused issues with the other
      // subsystems...
      if ((System.currentTimeMillis() - programStartTime / 1000) % delayTime == 0 && attemptReconnection != null
          && !attemptReconnection.isAlive() && !testConnection()) {
        // Update Networktable information periodically - TK
        setNetworktableStatus();

        try {
          if (attemptReconnection == null || !attemptReconnection.isAlive()) {
            attemptReconnection = new Thread(() -> this.attemptToReconnect());

            attemptReconnection.start();
          }
        } catch (IllegalThreadStateException e) {
          System.out
              .println("Exception occured in Camera: \n" + e + "\nThread state: " + attemptReconnection.getState());
        }
      }

      // Debugging Networktable Entries
      // aprilTagLocation tag = getAprilTagLocation(speakerAprilTag);
      // inst.getTable("Vision").getSubTable("Camera").getEntry("ID:
      // ").setInteger(tag.id);
      // inst.getTable("Vision").getSubTable("Camera").getEntry("Detected:
      // ").setBoolean(tag.isDetected);
      // inst.getTable("Vision").getSubTable("Camera").getEntry("Dist:
      // ").setDouble(tag.distance);
      // inst.getTable("Vision").getSubTable("Camera").getEntry("Ambiguity").setDouble(tag.ambiguity);
      // inst.getTable("Vision").getSubTable("Camera").getEntry("Degrees:
      // ").setDouble(tag.angle);
    } catch (Error e) {
      System.out.println("An error occured in Camera: \n" + e);
    }
  }

  public boolean getStatus() {
    setNetworktableStatus();

    // Just returns the boolean that shows connction status - TK
    if (versionMatches && connected) {
      return true;
    } else {
      return false;
    }
  }

  public PhotonTrackedTarget latestAprilTagDetection(int tag_id) {
    // Check if hasTargets is true, then check if the tag_id is equal to the
    if (april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == tag_id) {
          System.out.println("Tag ID: " + target.getFiducialId() + " X: " + target.getBestCameraToTarget().getX()
              + " Y: " + target.getBestCameraToTarget().getY());
          return target;
        }
      }
    }

    return null;
  }

  public boolean getAprilTagDetected() {
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return notes.getLatestResult().hasTargets();
    }
    return false;
  }

  public int getApriltagID() {
    // If this function returns a 0, that means there is not any detected targets
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return target.getFiducialId();
    } else {
      return -1;
    }
  }

  public double getApriltagYaw() {
    // If this function returns a 999, that means there is not any detected targets
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return target.getYaw();
    } else {
      return 999;
    }
  }

  public double getApriltagYaw(int id) {
    // If this function returns a 999, that means there is not any detected targets
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == id) {
          return target.getYaw();
        }
      }
      return 0;
    } else {
      return 999;
    }
  }

  public double getApriltagPitch() {
    // If this function returns a 999, that means there is not any detected targets
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return target.getPitch();
    } else {
      return 999;
    }
  }

  public double getApriltagPitch(int id) {
    // If this function returns a 999, that means there is not any detected targets
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == id) {
          return target.getPitch();
        }
      }
      return 0;
    } else {
      return 999;
    }
  }

  public DistAmb getApriltagDistX() {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets() && target != null) {
      return new DistAmb(target.getBestCameraToTarget().getY(), target.getPoseAmbiguity());
    } else {
      return null;
    }
  }

  public DistAmb getApriltagDistX(int id) {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == id) {
          return new DistAmb(target.getBestCameraToTarget().getY(), target.getPoseAmbiguity());
        }
      }
      return null;
    } else {
      return null;
    }
  }

  public DistAmb getApriltagDistY() {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets() && target != null) {
      return new DistAmb(target.getBestCameraToTarget().getX(), target.getPoseAmbiguity());
    } else {
      return null;
    }
  }

  public DistAmb getApriltagDistY(int id) {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == id) {
          return new DistAmb(target.getBestCameraToTarget().getX(), target.getPoseAmbiguity());
        }
      }
      return null;
    } else {
      return null;
    }
  }

  /**************************************************************************************/

  public double getAprilTagDist() {
    DistAmb distX = getApriltagDistX();
    DistAmb distY = getApriltagDistY();

    return Math.sqrt(
        (Math.pow(distX.distance, 2) + Math.pow(distY.distance, 2)));
  }

  public double getAprilTagDist(int id) {
    DistAmb distX = getApriltagDistX(id);
    DistAmb distY = getApriltagDistY(id);

    return Math.sqrt((Math.pow(distX.distance, 2)
        + Math.pow(distY.distance, 2)));
  }

  public double getDegToApriltag() {
    // Usable range of values with best consistancy: -50 - 50 With respect to
    // camera. - TK
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      // double targetYaw = getApriltagYaw();
      double requiredTurnDegrees;

      /*
       * Takes Photonvision Z angle theta value (3D processing mode on camera) and
       * gets sign, if sign is negative (aprilTag is on left of frame), it will turn
       * left the # of degs. that arcTan or inverse tan returns from the X & Y
       * coorinates. Else it turns right by the arcTan or inverse tan of the X & Y
       * coordinates. - TK
       */

      // Need to use the getX method that we wrote for Y in atan because it returns
      // the Photon Y. - TK
      requiredTurnDegrees = Math
          .toDegrees(Math.atan2(getApriltagDistX().distance, getApriltagDistY().distance));

      return requiredTurnDegrees;
    } else {
      return 999;
    }
  }

  public double getDegToApriltag(int id) {
    // Usable range of values with best consistancy: -50 - 50 With respect to
    // camera. - TK
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          double requiredTurnDegrees;

          /*
           * Takes Photonvision Z angle theta value (3D processing mode on camera) and
           * gets sign, if sign is negative (aprilTag is on left of frame), it will turn
           * left the # of degs. that arcTan or inverse tan returns from the X & Y
           * coorinates. Else it turns right by the arcTan or inverse tan of the X & Y
           * coordinates. - TK
           */

          // Need to use the getX method that we wrote for Y in atan because it returns
          // the Photon Y. - TK
          requiredTurnDegrees = Math
              .toDegrees(Math.atan2(getApriltagDistX(id).distance, getApriltagDistY(id).distance));

          return requiredTurnDegrees;
        }
      }
      return 999;
    } else {
      return 9999;
    }
  }

  public aprilTagLocation getAprilTagLocation(int id) {
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          DistAmb measurement = getApriltagDistY(id);
          if (measurement != null) {
            double deg = getDegToApriltag(id);

            return new aprilTagLocation(true, measurement.distance, measurement.ambiguity, deg, id);
          } else {
            return new aprilTagLocation(false, 0, 0, 0, 0);
          }
        }
      }
    }
    return new aprilTagLocation(false, 0, 0, 0, -1);
  }

  public aprilTagLayout getAprilTagLayout() {
    // TODO: Check to see if this variable matches the one I just added for the
    // photonvision pose estimator - TK
    return aprilTagLayout;
  }

  public boolean getNoteDetected() {
    if (connected && versionMatches && notes != null && notes.getLatestResult().hasTargets()) {
      return notes.getLatestResult().hasTargets();
    }
    return false;
  }

  public double getNoteAngle() {
    PhotonTrackedTarget target = notes.getLatestResult().getBestTarget();

    // Robot relative angle
    if (connected && versionMatches && notes != null && notes.getLatestResult().hasTargets() && target != null) {
      return target.getYaw();
    }
    return 999;
  }

  public double getNoteArea() {
    PhotonTrackedTarget target = notes.getLatestResult().getBestTarget();

    if (connected && versionMatches && notes != null && notes.getLatestResult().hasTargets() && target != null) {
      return target.getArea();
    }
    return 0.0;
  }

  public double getTimestamp() {
    return inst.getTable("photonvision").getSubTable("april").getEntry("heartbeat").getDouble(0);
  }

  public double getCurrentTime() {
    // Shouldn't need to typecast here, but JIC :) - TK
    return (double) (System.currentTimeMillis() - programStartTime);
  }

  public double getLatencySeconds() {
    return april.getLatestResult().getTimestampSeconds() / 1000;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    aprilTagPoseEstimator.setReferencePose(SwerveDrive.getInstance().getPose());
    return aprilTagPoseEstimator.update(april.getLatestResult());
  }

  public Optional<EstimatedRobotPose> getSideEstimatedGlobalPose() {
    sideAprilTagPoseEstimator.setReferencePose(SwerveDrive.getInstance().getPose());
    return sideAprilTagPoseEstimator.update(sideApril.getLatestResult());
  }

  // public SequentialCommandGroup pathfindToAprilTag() {
  // SequentialCommandGroup goToAprilTag = new SequentialCommandGroup(
  // new turnToFaceApriltag(speakerAprilTag, RobotContainer.swerve,
  // Camera.getInstance()),
  // new InstantCommand(() -> {
  // currentSwervePose2d = RobotContainer.swerve.getPose();
  // currentX = currentSwervePose2d.getX();
  // currentY = currentSwervePose2d.getY();
  // newX = getApriltagDistX(speakerAprilTag);
  // newY = percentTravelDist * getApriltagDistY(speakerAprilTag);
  // }),
  // new pathfindToApriltag(new Pose2d((currentX - newX), (currentY - newY), new
  // Rotation2d(0)),
  // Camera.getInstance(),
  // RobotContainer.swerve),
  // new turnToFaceApriltag(speakerAprilTag, RobotContainer.swerve,
  // Camera.getInstance()));

  // Fallback code. NOT tested!!!!! - TK
  // SequentialCommandGroup goToAprilTag = new SequentialCommandGroup(
  // new InstantCommand(() -> swerveDrive.resetPose(new Pose2d(4.5, 6.5, new
  //
  // Rotation2d(Math.toRadians(swerveDrive.getPose().getRotation().getDegrees()))))),
  // new InstantCommand(() -> System.out.println("X: " +
  // swerveDrive.getPose().getX() + " Y: " + swerveDrive.getPose().getY())),
  // // new Pathfinding(new Pose2d((swerveDrive.getPose().getX()/* + 0.25*/),
  // (swerveDrive.getPose().getY()/* + 0.25*/), new
  // Rotation2d(Math.toRadians(swerveDrive.getPose().getRotation().getDegrees()
  // +
  // 45))), Camera.getInstance(), swerveDrive),
  // new InstantCommand(() -> System.out.println("DistX to Target:
  // "+getApriltagDistX(2))),
  // // new InstantCommand(() -> RobotContainer.m_robotDrive.drive(0,
  // Math.signum(getApriltagDistX(2)), getDegToApriltag(2), false)),
  // new InstantCommand(() ->
  // RobotContainer.swerve.drive(0,1.0*Math.signum(getApriltagDistX(2)), 0,
  // false)),
  // new InstantCommand(() -> System.out.println("DONE"))
  // );

  // return goToAprilTag;
  // }

  // public SequentialCommandGroup pathfindToAprilTag(int id) {
  // SequentialCommandGroup goToAprilTag = new SequentialCommandGroup(
  // new turnToFaceApriltag(id, swerveDrive, Camera.getInstance()),
  // new InstantCommand(() -> {
  // currentSwervePose2d = swerveDrive.getPose();
  // currentX = currentSwervePose2d.getX();
  // currentY = currentSwervePose2d.getY();
  // newX = getApriltagDistX(id);
  // newY = percentTravelDist * getApriltagDistY(id);
  // }),
  // new pathfindToApriltag(new Pose2d((currentX - newX), (currentY - newY), new
  // Rotation2d(0)),
  // Camera.getInstance(),
  // swerveDrive),
  // new turnToFaceApriltag(swerveDrive, Camera.getInstance()));

  // return goToAprilTag;
  // }
}