// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.Camera;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive extends SubsystemBase {
  private static SwerveDrive instance = new SwerveDrive();
  ProfiledPIDController thetaController = new ProfiledPIDController(2, 0, .1, new Constraints(360, 720));
  SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
  private final SwerveDrivePoseEstimator poseEstimator;
  // private Camera camera = Camera.getInstance();
  public static AHRS gyro;

  private final Translation2d[] locations = {
      new Translation2d(Constants.Bot.botLength, Constants.Bot.botLength),
      new Translation2d(Constants.Bot.botLength, -Constants.Bot.botLength),
      new Translation2d(-Constants.Bot.botLength, Constants.Bot.botLength),
      new Translation2d(-Constants.Bot.botLength, -Constants.Bot.botLength)
  };

  SwerveModule[] modules = {
      new SwerveModule(
          "frontLeft",
          Constants.SensorIDs.FL,
          Constants.MotorIDs.FLVortex,
          Constants.MotorIDs.FLNeo,
          Constants.Bot.FLBaseAngle),
      new SwerveModule(
          "frontRight",
          Constants.SensorIDs.FR,
          Constants.MotorIDs.FRVortex,
          Constants.MotorIDs.FRNeo,
          Constants.Bot.FRBaseAngle),
      new SwerveModule("backLeft",
          Constants.SensorIDs.BL,
          Constants.MotorIDs.BLVortex,
          Constants.MotorIDs.BLNeo,
          Constants.Bot.BLBaseAngle),
      new SwerveModule("backRight",
          Constants.SensorIDs.BR,
          Constants.MotorIDs.BRVortex,
          Constants.MotorIDs.BRNeo,
          Constants.Bot.BRBaseAngle)
  };

  double odometryOffset = 0;

  private ChassisSpeeds botSpeeds = new ChassisSpeeds(0, 0, 0);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      locations[0], locations[1], locations[2], locations[3]);

  public static SwerveDrive getInstance() {
    if (instance == null) {
      instance = new SwerveDrive();
    }
    return instance;
  }

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used
   * below are robot specific, and should be tuned.
   */

  public SwerveDrive() {
    NetworkTableInstance.getDefault().getTable("VisionStdDev").getEntry("VisionstdDev").setDouble(.01);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.PI / 45); // 4 degrees
    gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro.reset();

    // Autobuilder for Pathplanner Goes last in constructor! TK
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(Constants.PathplannerConstants.TransP, Constants.PathplannerConstants.TransI,
                Constants.PathplannerConstants.TransD), // Translation PID constants
            new PIDConstants(Constants.PathplannerConstants.RotP, Constants.PathplannerConstants.RotI,
                Constants.PathplannerConstants.RotD) // Rotation PID constants
        ),
        Constants.PathplannerConstants.config, // The robot configuration
        this::shouldFlipPath,
        this // Reference to this subsystem to set requirements
    );
    // std deviation taken from examples
    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            modules[0].getSwerveModulePosition(),
            modules[1].getSwerveModulePosition(),
            modules[2].getSwerveModulePosition(),
            modules[3].getSwerveModulePosition()
        },
        new Pose2d(),
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(2)),
        VecBuilder.fill(1.5, 1.5, Units.degreesToRadians(30)));

    Logger.recordOutput("Actual States", states);
    Logger.recordOutput("Gyro States", states);
    Logger.recordOutput("Set States", swerveModuleStates);
    Logger.recordOutput("Odometry", poseEstimator.getEstimatedPosition());
  }

  // WPILib
  StructArrayPublisher<SwerveModuleState> actualStates = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Actual States", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> gyroStates = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Gyro States", SwerveModuleState.struct).publish();

  StructArrayPublisher<SwerveModuleState> setStates = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Set States", SwerveModuleState.struct).publish();
  StructPublisher<Pose2d> odometryStruct = NetworkTableInstance.getDefault()
      .getStructTopic("Odometry", Pose2d.struct).publish();
  SwerveModuleState[] states = new SwerveModuleState[4];

  public void periodic() {
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }

    updateOdometry();
    actualStates.set(swerveModuleStates);
    setStates.set(states);
    gyroStates.set(kinematics
        .toSwerveModuleStates(new ChassisSpeeds(gyro.getVelocityX(), gyro.getVelocityY(), gyro.getRawGyroX())));
    // double visionStdDev =
    // NetworkTableInstance.getDefault().getTable("VisionStdDev").getEntry("VisionstdDev")
    // .getDouble(.02);
    // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionStdDev,
    // visionStdDev, Units.degreesToRadians(30)));
    // poseEstimator.addVisionMeasurement(camera.getEstimatedGlobalPose(),
    // camera.getTimestamp());
    odometryStruct.set(getPose());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    botSpeeds = ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed, rot), .02);
    swerveModuleStates = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            .02));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Bot.maxModuleSpeed);

    for (int i = 0; i < 4; i++) {
      modules[i].setStates(swerveModuleStates[i], false);
    }
  }

  // For Pathplanner
  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  public void driveRobotAtAngle(double xSpeed, double ySpeed, double angleRadians, boolean fieldRelative) {
    thetaController.setGoal(angleRadians);
    double angularSpeed = thetaController.calculate(getPose().getRotation().getRadians());
    drive(xSpeed, ySpeed, angularSpeed, fieldRelative);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    poseEstimator.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            modules[0].getSwerveModulePosition(),
            modules[1].getSwerveModulePosition(),
            modules[2].getSwerveModulePosition(),
            modules[3].getSwerveModulePosition()
        });

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.

    try {

      if (Camera.getInstance().getStatus()) {
        // TODO: The following line was from the main branch but was not in the latest camera vesrion
        // Optional<EstimatedRobotPose> sidePose = Camera.getInstance().getSideEstimatedGlobalPose();
        Optional<EstimatedRobotPose> pose = Camera.getInstance().getEstimatedGlobalPose();
        // DistAmb reading = Camera.getInstance().getApriltagDistX();
        if (pose.isPresent()) {
          System.out.println("Updating as expected");
          poseEstimator.addVisionMeasurement(pose.get().estimatedPose.toPose2d(),
              Timer.getFPGATimestamp() - .04);
          // System.out.println("Target Detected");
        } else {
          System.out.println("No Pose");
        }
        // if (sidePose.isPresent()) {
        //   poseEstimator.addVisionMeasurement(sidePose.get().estimatedPose.toPose2d(),
        //       Timer.getFPGATimestamp() - .04);

        // }
      } else {
        System.out.println("Camera not detected");
      }

    } catch (Error test) {
      System.err.println(test);
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getSwerveModulePosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public boolean shouldFlipPath() {
    return DriverStation.getAlliance().get().equals(Alliance.Red);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getExpectedPose() {
    return getPose().plus(new Transform2d(
        new Translation2d(botSpeeds.vxMetersPerSecond * .05, botSpeeds.vyMetersPerSecond * .05), new Rotation2d()));
  }

  public void setVisionStdDeviations(double deviation) {
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(deviation, deviation, Units.degreesToRadians(30)));
  }

  private PIDController turnController = new PIDController(0.025, 0, 0.0025);

  public void turnToAprilTag(int ID) {
    // TODO: Potential null error unhandled here
    // turnPID.enableContinuousInput(0, 360);
    try {
      double botAngle = getPose().getRotation().getDegrees();
      double offsetAngle = Camera.getInstance().getDegToApriltag(ID);
      double setpoint = 0;
      if (botAngle - offsetAngle <= 0)
        setpoint = botAngle + offsetAngle;
      else
        setpoint = botAngle - offsetAngle;

      turnController.setSetpoint(setpoint);
      drive(0, 0, turnController.calculate(botAngle), false);

    } catch (Exception e) {
      System.err.println(e.getLocalizedMessage());
    }
  }

  public void resetGyro() {
    odometryOffset += gyro.getAngle();
    gyro.reset();
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }
}