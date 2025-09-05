// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.NetworkTables;
import frc.robot.subsystems.odometry.Odometry;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive extends SubsystemBase {

  private static SwerveDrive instance = SwerveDrive.getInstance();
  ProfiledPIDController thetaController = new ProfiledPIDController(2, 0, .1, new Constraints(360, 720));
  SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
  // private Camera camera = Camera.getInstance();
  public static Odometry odometry;

  public final Translation2d[] locations = {
      new Translation2d(Constants.Bot.botLength, Constants.Bot.botLength),
      new Translation2d(Constants.Bot.botLength, -Constants.Bot.botLength),
      new Translation2d(-Constants.Bot.botLength, Constants.Bot.botLength),
      new Translation2d(-Constants.Bot.botLength, -Constants.Bot.botLength)
  };

  public final SwerveModule[] modules = {
      new SwerveModule(
          "frontLeft",
          Constants.SensorIDs.FL,
          Constants.MotorIDs.FLVortex,
          Constants.MotorIDs.FLNeo,
          Constants.Bot.FLZeroOffset,
          false),
      new SwerveModule(
          "frontRight",
          Constants.SensorIDs.FR,
          Constants.MotorIDs.FRVortex,
          Constants.MotorIDs.FRNeo,
          Constants.Bot.FRZeroOffset,
          true),
      new SwerveModule("backLeft",
          Constants.SensorIDs.BL,
          Constants.MotorIDs.BLVortex,
          Constants.MotorIDs.BLNeo,
          Constants.Bot.BLZeroOffset,
          false),
      new SwerveModule("backRight",
          Constants.SensorIDs.BR,
          Constants.MotorIDs.BRVortex,
          Constants.MotorIDs.BRNeo,
          Constants.Bot.BRZeroOffset,
          true)
  };

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      locations[0], locations[1], locations[2], locations[3]);

  public static SwerveDrive getInstance() {
    if (instance == null) {
      instance = new SwerveDrive();
    }
    return instance;
  }

  private SwerveDrive() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.PI / 45); // 4 degrees
    odometry = Odometry.getInstance();

    NetworkTables.maxVelo.setDouble(Constants.Bot.maxChassisSpeed);

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        odometry::getPose, // Robot pose supplier
        odometry::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants( // Translation PID constants
                Constants.PathplannerConstants.TransP,
                Constants.PathplannerConstants.TransI,
                Constants.PathplannerConstants.TransD),
            new PIDConstants( // Rotation PID constants
                Constants.PathplannerConstants.RotP,
                Constants.PathplannerConstants.RotI,
                Constants.PathplannerConstants.RotD)),
        config, // The robot configuration
        this::shouldFlipPath,
        this // Reference to this subsystem to set requirements
    );
  }

  public void periodic() {
    odometry.update();
    updateNetworktables();
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
    ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed, rot), .02);
    swerveModuleStates = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, odometry.getGyroRotation())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            .02));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Bot.maxChassisSpeed);

    for (int i = 0; i < 4; i++) {
      modules[i].setStates(swerveModuleStates[i]);
    }
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  public boolean shouldFlipPath() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      double distanceMeters = modules[i].driveEncoder.getPosition() * Constants.Bot.encoderRotationToMeters;
      Rotation2d angle = Rotation2d.fromDegrees(modules[i].getTurnEncoder().getAbsolutePosition());
      positions[i] = new SwerveModulePosition(distanceMeters, angle);
    }
    return positions;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  private void updateNetworktables() {
    if (swerveModuleStates != null) {
      ArrayList<Double> desiredStates = new ArrayList<>(8);

      for (int i = 0; i < swerveModuleStates.length; i++) {
        // Angle, Velocity / Module
        if (swerveModuleStates[i] != null) {
          desiredStates.add(swerveModuleStates[i].angle != null ? swerveModuleStates[i].angle.getDegrees() : 0);
          desiredStates.add(-swerveModuleStates[i].speedMetersPerSecond);
        } else {
          desiredStates.add(0.0);
          desiredStates.add(0.0);
        }
      }

      NetworkTables.desiredSwerveStates_da
          .setDoubleArray(desiredStates.stream().mapToDouble(Double::doubleValue).toArray());

      ArrayList<Double> measuredStates = new ArrayList<>(8);

      for (int i = 0; i < modules.length; i++) {
        // Angle, Velocity / Module
        measuredStates.add(modules[i].getTurnEncoder().getAbsolutePosition());
        measuredStates.add(-modules[i].getVelocity());
      }

      NetworkTables.measuredSwerveStates_da
          .setDoubleArray(measuredStates.stream().mapToDouble(Double::doubleValue).toArray());
    }

    NetworkTables.botRotDeg_d.setDouble(odometry.getGyroRotation().getDegrees());
  }

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean locked) {
    if (states.length == 4) {
      for (int i = 0; i < 4; i++) {
        if (locked) {
          states[i].angle = Rotation2d.fromDegrees(Constants.Bot.lockedAngles[i]);
          states[i].speedMetersPerSecond = 0;
          swerveModuleStates[i] = states[i];
        }

        modules[i].setStates(states[i]);
      }
    } else {
      System.err.println("To many or too few swerve module states. NOT SETTING!");
    }
  }
}