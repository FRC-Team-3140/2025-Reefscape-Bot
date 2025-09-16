// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.libs.FieldAprilTags;
import frc.robot.libs.FlipPose;

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
  public static final boolean replay = true;
  //public static final Logger.SimulationMode simMode = Logger.SimulationMode.REPLAY;
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
    public static final double gearRatio = 6.75;
    public static final double steerGearRatio = 150 / 7;
    public static final double botMass = 48.988;
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double botLength = Units.inchesToMeters(29);

    // In meters per second, determined from the free speed of the bot via
    // SwerveDriveSpecialties
    public static final double maxChassisSpeed = 6;
    public static final double maxModuleSpeed = maxChassisSpeed / wheelDiameter / Math.PI;
    public static final double maxTurnSpeed = Double.MAX_VALUE; // These are basically infinite for our purposes
    public static final double maxAcceleration = 2500;
    public static final double botRadius = Math.hypot(botLength, botLength);
    // Max Speed divided by the circumference a circle determined by the distance of
    // the module from the center, divided by 2 pi to convert to radians
    public static final double maxChassisTurnSpeed = maxChassisSpeed / botRadius;
    public static final double encoderRotationToMeters = Math.PI * wheelDiameter / gearRatio;

    public static final double elevatorEncoderDegreesToMeters = 0.001;

    // Swerve Module Base Angles
    public static final double FLZeroOffset = 216.997730;// 217.720;

    public static final double FRZeroOffset = 136.602900;// 228.319;

    public static final double BLZeroOffset = 200.317267;// 197.621;

    public static final double BRZeroOffset = 310.841840;// 312.425;

    public static final double[] lockedAngles = {
        45,
        315,
        315,
        45
    };

    // Default swerve state
    // new SwerveModuleState initializes states with 0s for angle and velocity
    public static final SwerveModuleState[] defaultSwerveStates = {
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0))
    };

  }

  public static class Limits {
    // Elevator
    public static final double ElevMovement = 0.2;
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
    // public static final double maxTimeBeteweenFrames = 0.1;
    public static final double frontOffsetToCenter = Units.inchesToMeters(13);
    public static final double backOffsetToCenter = -Units.inchesToMeters(13);
    public static final double backOffsetToCenterVert = -Units.inchesToMeters(19);
    public static final double maxDistCutoff = 2;
    public static final double minAmbiguity = 0.05;
  }

  public static class PathplannerConstants {
    public static RobotConfig config;

    // Field Dimensions
    public static final double FieldLength = 17.548;
    public static final double FieldWidth = 8.052;

    // Translation PID Values
    public static final double TransP = 12;
    public static final double TransI = 0;
    public static final double TransD = 0;

    // Rotation PID Values
    public static final double RotP = 5.0;
    public static final double RotI = 0;
    public static final double RotD = 0;

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
    public static final double processorScoreBottom = 0.75;
    public static final double processorScoreTop = stowedAlgaeTop;
  }

  public static class ElevatorHeights {
    public static final double minimum = 0;
    public static final double maximum = 10670 * Bot.elevatorEncoderDegreesToMeters;

    public static final double groudAlgaeHeight = 2;

    public static final double reefAlgaeL1Height = 4.55;
    public static final double reefAlgaeL2Height = 7.3;
    public static final double reefCoralL1Height = 3.5;
    public static final double reefCoralL2Height = 4.5;
    public static final double reefCoralL3Height = 7.054625;
    public static final double reefCoralL4Height = maximum;

    public static final double sourceIntake = 2; // orig 1.75
    // the above value was used at RCR

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
    public static final String Debug = "Debug";
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
    public static Pose2d getPose(int side, int pos) { // pos: -1 left, 0 center, 1 right
      double sideOffset = 0.17 + (pos * 0.02);
      double backOffset = 0.5;
      int id = switch (side) {
        case 0 -> 18;
        case 1 -> 19;
        case 2 -> 20;
        case 3 -> 21;
        case 4 -> 22;
        case 5 -> 17;
        default -> -1;
      };
      if (id == -1)
        return null;
      Pose2d tagPose = FieldAprilTags.getInstance().getTagPose(id);
      double theta = tagPose.getRotation().getRadians();
      Translation2d offsetTranslation = tagPose.getTranslation();
      if (pos != 0)
        offsetTranslation = offsetTranslation.plus(new Translation2d(sideOffset * Math.cos(theta + Math.PI / 2 * pos),
            sideOffset * Math.sin(theta + Math.PI / 2 * pos)));
      offsetTranslation = offsetTranslation.plus(
          new Translation2d(backOffset * Math.cos(theta), backOffset * Math.sin(theta)));

      Pose2d rawPose = new Pose2d(offsetTranslation.getX(), offsetTranslation.getY(), new Rotation2d(theta - Math.PI));
      return FlipPose.flipIfRed(rawPose);
    }

    public ReefPoses() {
    }
  }
}