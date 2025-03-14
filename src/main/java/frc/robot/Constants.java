// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.libs.FieldAprilTags;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class MotorIDs {
    /* Swerve Drive Motors: */
    // FL
    public static final int FLNeo = 1;
    public static final int FLVortex = 2;

    // FR
    public static final int FRNeo = 3;
    public static final int FRVortex = 4;

    // BL
    public static final int BLNeo = 5;
    public static final int BLVortex = 6;

    // BR
    public static final int BRNeo = 7;
    public static final int BRVortex = 8;

    // Elevator
    public static final int ElevLNeo = 9;
    public static final int ElevRNeo = 10;

    // End Effector
    public static final int EELeft = 11;
    public static final int EERight = 12;
    public static final int EETop = 13;

    // Algae Intake
    public static final int AIRotate = 14;
    public static final int AIIntake = 15;
  }

  public static class SensorIDs {
    // Swerve Modules
    public static final int FL = 0;

    public static final int FR = 1;

    public static final int BL = 2;

    public static final int BR = 3;

    // Elevator

    public static final int ElevEncoderRightA = 2;
    public static final int ElevEncoderRightB = 3;
    public static final int ElevEncoderLeftA = 4;
    public static final int ElevEncoderLeftB = 5;

    // Algae Intake
    public static final int AIEncoder = 0;

    // End Effector
    public static final int EECoralSensor = 1;

  }

  public static class Bot {
    public static final double gearRatio = 6.12;
    public static final double botMass = 48.988;
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double botLength = Units.inchesToMeters(29);

    // In meters per second, determined from the free speed of the bot via
    // SwerveDriveSpecialties
    public static final double maxChassisSpeed = 5.8969405214990;
    public static final double maxModuleSpeed = maxChassisSpeed / wheelDiameter / Math.PI;
    public static final double maxTurnSpeed = Double.MAX_VALUE; // These are basically infinite for our purposes
    public static final double maxAcceleration = 4000;
    public static final double botRadius = Math.hypot(botLength, botLength);
    // Max Speed divided by the circumference a circle determined by the distance of
    // the module from the center, divided by 2 pi to convert to radians
    public static final double maxChassisTurnSpeed = maxChassisSpeed / botRadius;
    public static final double encoderRotationToMeters = 2 * Math.PI * ((wheelDiameter / 2) / gearRatio);

    public static final double elevatorEncoderDegreesToMeters = 0.001;

    // Swerve Module Base Angles
    public static final double FLZeroOffset = 217.720;

    public static final double FRZeroOffset = 228.319;

    public static final double BLZeroOffset = 197.621;

    public static final double BRZeroOffset = 312.425;

    public static final double[] lockedAngles = {
        45,
        315,
        315,
        45
    };

    // Default swerve state
    // new SwerveModuleState initializes states with 0s for angle and velocity
    public static final SwerveModuleState[] defaultSwerveStates = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

  }

  public static class Limits {
    // Elevator
    public static final double ElevMovement = 0.1;
    public static final double CurrentHomeThreshold = 4;
    public static final double ElevPosThreshold = 0.5;

    // Endeffector
    public static final double EEAlgaeIntakeCurrentThreshold = 20;
  }

  public static class Controller {
    public static final int DriverControllerPort = 0;
    public static final int SecondaryDriverControllerPort = 1;

    public static final double triggerThreshold = 0.3;
  }

  public static class Constraints {
    public static final double elevatorMaxVelocity = 1;
    public static final double elevatorMaxAcceleration = 3;
  }

  public static class CameraConstants {
    public static final double maxTimeBeteweenFrames = 0.1;
    public static final double aprilOffsetToCenter0 = Units.inchesToMeters(13);
    public static final double aprilOffsetToCenter2 = Units.inchesToMeters(13);
    public static final double maxValidDistance = 3;
  }

  public static class PathplannerConstants {
    public static RobotConfig config;

    // Translation PID Values
    public static final double TransP = 5.0;
    public static final double TransI = 1.0;
    public static final double TransD = 0.2;

    // Rotation PID Values
    public static final double RotP = 5.0;
    public static final double RotI = 1.0;
    public static final double RotD = 0.2;

    public static final PathConstraints pathplannerConstraints = new PathConstraints(
        Constants.Bot.maxChassisSpeed,
        4.0,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720));
  }

  public static class AlgaeIntakeAngles {
    public static final double min = 0.45;
    public static final double max = 0.96;
    public static final double stowed = max;
    public static final double stowedAlgaeTop = 0;
    public static final double stowedAlgaeBottom = 0;
    public static final double groundIntake = 0.7;
    public static final double reefIntake = 0.8;
    public static final double processorScoreBottom = 0.8;
    public static final double processorScoreTop = stowedAlgaeTop;
  }

  public static class ElevatorHeights {
    public static final double minimum = 0;
    public static final double maxiumum = 10900 * Bot.elevatorEncoderDegreesToMeters;

    public static final double groudAlgaeHeight = 2;

    public static final double reefAlgaeL1Height = 4.55;
    public static final double reefAlgaeL2Height = 7.3;
    public static final double reefCoralL1Height = 3.5;
    public static final double reefCoralL2Height = 4.5;
    public static final double reefCoralL3Height = 7.054625;
    public static final double reefCoralL4Height = maxiumum;

    public static final double sourceIntake = 1.75;

    public static final double processerHeight = 0;
    public static final double safeStowed = 0;

    public static final double homeUpDist = 1;
  }

  public static class Odometry {
    public static final double TagCorrectionSpeed = 0.75;
  }

  public static class NetworktablePaths {
    public static final String Dashboard = "Dashboard";

    public static final String Sensors = "sensors3140";

    // Subtables of Dashboard
    public static final String DS = "DS";
    public static final String Voltage = "Voltage";
    public static final String Pose = "Pose";
    public static final String Reef = "Reef";
    public static final String Test = "Dev";
    public static final String Misc = "Misc";
  }

  public static class MotorSpeeds {
    public static class EndEffector {
      public static final double beltIntake = 0.75;
      public static final double manipulatorIntake = 0.75;
      public static final double manipulatorScore = 0.25;
      public static final double manipulatorScoreL4 = 0.1;
      public static final double algaeIntakeSpeed = 0.25;
      public static final double algaeProcessorSpeed = 0.25;
    }
  }

  public static class LED {
    public static final int Port = 0;
    public static final int LEDCount = 76;
    public static final int RainbowSpan = 5;
  }

  public static class ReefPoses {
    public HashMap<Integer, Pose2d> reefCoralPosesBlue = new HashMap<>();
    public HashMap<Integer, Pose2d> reefAlgaePosesBlue = new HashMap<>();
    public HashMap<Integer, Pose2d> reefCoralPosesRed = new HashMap<>();
    public HashMap<Integer, Pose2d> reefAlgaePosesRed = new HashMap<>();

    public ReefPoses() {
      /////////////////////////// Blue Alliance ///////////////////////////
      /// Coral
      reefCoralPosesBlue.put(0, new Pose2d(3.915, 3.85, new Rotation2d(Units.degreesToRadians(0))));
      reefCoralPosesBlue.put(1, new Pose2d(3.915, 4.18, new Rotation2d(Units.degreesToRadians(0))));
      reefCoralPosesBlue.put(2, new Pose2d(3.683, 5.062, new Rotation2d(Units.degreesToRadians(-60))));
      reefCoralPosesBlue.put(3, new Pose2d(3.978, 5.245, new Rotation2d(Units.degreesToRadians(-60))));
      reefCoralPosesBlue.put(4, new Pose2d(4.994, 5.235, new Rotation2d(Units.degreesToRadians(-120))));
      reefCoralPosesBlue.put(5, new Pose2d(5.299, 5.082, new Rotation2d(Units.degreesToRadians(-120))));
      reefCoralPosesBlue.put(6, new Pose2d(5.7787, 4.18, new Rotation2d(Units.degreesToRadians(180))));
      reefCoralPosesBlue.put(7, new Pose2d(5.7787, 3.85, new Rotation2d(Units.degreesToRadians(180))));
      reefCoralPosesBlue.put(8, new Pose2d(5.289, 2.988, new Rotation2d(Units.degreesToRadians(120))));
      reefCoralPosesBlue.put(9, new Pose2d(5, 2.805, new Rotation2d(Units.degreesToRadians(120))));
      reefCoralPosesBlue.put(10, new Pose2d(3.967, 2.815, new Rotation2d(Units.degreesToRadians(60))));
      reefCoralPosesBlue.put(11, new Pose2d(3.693, 2.978, new Rotation2d(Units.degreesToRadians(60))));

      /// Algae
      reefAlgaePosesBlue.put(0, new Pose2d(
          FieldAprilTags.getInstance().blueReefTagsHash.get(0).pose.getX(),
          FieldAprilTags.getInstance().blueReefTagsHash.get(0).pose.getY(),
          new Rotation2d(Units.degreesToRadians(0))));
      reefAlgaePosesBlue.put(1, new Pose2d(
          FieldAprilTags.getInstance().blueReefTagsHash.get(1).pose.getX(),
          FieldAprilTags.getInstance().blueReefTagsHash.get(1).pose.getY(),
          new Rotation2d(Units.degreesToRadians(-60))));
      reefAlgaePosesBlue.put(2, new Pose2d(
          FieldAprilTags.getInstance().blueReefTagsHash.get(2).pose.getX(),
          FieldAprilTags.getInstance().blueReefTagsHash.get(2).pose.getY(),
          new Rotation2d(Units.degreesToRadians(-120))));
      reefAlgaePosesBlue.put(3, new Pose2d(
          FieldAprilTags.getInstance().blueReefTagsHash.get(3).pose.getX(),
          FieldAprilTags.getInstance().blueReefTagsHash.get(3).pose.getY(),
          new Rotation2d(Units.degreesToRadians(180))));
      reefAlgaePosesBlue.put(4, new Pose2d(
          FieldAprilTags.getInstance().blueReefTagsHash.get(4).pose.getX(),
          FieldAprilTags.getInstance().blueReefTagsHash.get(4).pose.getY(),
          new Rotation2d(Units.degreesToRadians(120))));
      reefAlgaePosesBlue.put(5, new Pose2d(
          FieldAprilTags.getInstance().blueReefTagsHash.get(5).pose.getX(),
          FieldAprilTags.getInstance().blueReefTagsHash.get(5).pose.getY(),
          new Rotation2d(Units.degreesToRadians(60))));

      /////////////////////////// Red Alliance ///////////////////////////
      /// Coral
      reefCoralPosesRed.put(0, new Pose2d(14.365, 4.18, new Rotation2d(Units.degreesToRadians(180))));
      reefCoralPosesRed.put(1, new Pose2d(14.365, 3.85, new Rotation2d(Units.degreesToRadians(180))));
      reefCoralPosesRed.put(2, new Pose2d(13.847, 2.988, new Rotation2d(Units.degreesToRadians(120))));
      reefCoralPosesRed.put(3, new Pose2d(13.543, 2.805, new Rotation2d(Units.degreesToRadians(120))));
      reefCoralPosesRed.put(4, new Pose2d(12.556, 2.815, new Rotation2d(Units.degreesToRadians(60))));
      reefCoralPosesRed.put(5, new Pose2d(12.261, 2.978, new Rotation2d(Units.degreesToRadians(60))));
      reefCoralPosesRed.put(6, new Pose2d(11.763, 3.85, new Rotation2d(Units.degreesToRadians(0))));
      reefCoralPosesRed.put(7, new Pose2d(11.763, 4.18, new Rotation2d(Units.degreesToRadians(0))));
      reefCoralPosesRed.put(8, new Pose2d(12.25, 5.062, new Rotation2d(Units.degreesToRadians(-60))));
      reefCoralPosesRed.put(9, new Pose2d(12.54, 5.245, new Rotation2d(Units.degreesToRadians(-60))));
      reefCoralPosesRed.put(10, new Pose2d(13.572, 5.235, new Rotation2d(Units.degreesToRadians(-120))));
      reefCoralPosesRed.put(11, new Pose2d(13.867, 5.082, new Rotation2d(Units.degreesToRadians(-120))));

      /// Algae
      reefAlgaePosesRed.put(0, new Pose2d(
          FieldAprilTags.getInstance().redReefTagsHash.get(0).pose.getX(),
          FieldAprilTags.getInstance().redReefTagsHash.get(0).pose.getY(),
          new Rotation2d(Units.degreesToRadians(0))));
      reefAlgaePosesRed.put(1, new Pose2d(
          FieldAprilTags.getInstance().redReefTagsHash.get(1).pose.getX(),
          FieldAprilTags.getInstance().redReefTagsHash.get(1).pose.getY(),
          new Rotation2d(Units.degreesToRadians(-60))));
      reefAlgaePosesRed.put(2, new Pose2d(
          FieldAprilTags.getInstance().redReefTagsHash.get(2).pose.getX(),
          FieldAprilTags.getInstance().redReefTagsHash.get(2).pose.getY(),
          new Rotation2d(Units.degreesToRadians(-120))));
      reefAlgaePosesRed.put(3, new Pose2d(
          FieldAprilTags.getInstance().redReefTagsHash.get(3).pose.getX(),
          FieldAprilTags.getInstance().redReefTagsHash.get(3).pose.getY(),
          new Rotation2d(Units.degreesToRadians(180))));
      reefAlgaePosesRed.put(4, new Pose2d(
          FieldAprilTags.getInstance().redReefTagsHash.get(4).pose.getX(),
          FieldAprilTags.getInstance().redReefTagsHash.get(4).pose.getY(),
          new Rotation2d(Units.degreesToRadians(120))));
      reefAlgaePosesRed.put(5, new Pose2d(
          FieldAprilTags.getInstance().redReefTagsHash.get(5).pose.getX(),
          FieldAprilTags.getInstance().redReefTagsHash.get(5).pose.getY(),
          new Rotation2d(Units.degreesToRadians(60))));
    }
  }
}