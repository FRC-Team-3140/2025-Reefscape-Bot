// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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

    // Ground Intake
    public static final int GILeft = 16;
    public static final int GIRight = 17;
    public static final int GILift = 18;
  }

  public static class SensorIDs {
    // Swerve Modules
    public static final int FL = 0;

    public static final int FR = 1;

    public static final int BL = 2;

    public static final int BR = 3;

    // Elevator
    public static final int ElevEncoder = 4;

    // Algae Intake
    public static final int AIEncoder = 5;

    // Ground Intake
    public static final int GIEncoder = 6;

    public static final int EECoralSensor = 7;

    // Reef Alignment
    public static final int[] distanceSensorArray = { 8, 9, 10, 11 };
  }

  public static class Bot {
    public static final double gearRatio = 6.12;
    public static final double botMass = 24.4; // TODO: Update Bot Mass
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double botLength = Units.inchesToMeters(29);

    // In meters per second, determined from the free speed of the bot via
    // SwerveDriveSpecialties
    public static final double maxChassisSpeed = 11.8;
    public static final double maxModuleSpeed = maxChassisSpeed / wheelDiameter / Math.PI;
    public static final double maxTurnSpeed = Double.MAX_VALUE; // These are basically infinite for our purposes
    public static final double maxAcceleration = 4000;
    public static final double botRadius = Math.hypot(botLength, botLength);
    // Max Speed divided by the circumference a circle determined by the distance of
    // the module from the center, divided by 2 pi to convert to radians
    public static final double maxChassisTurnSpeed = maxChassisSpeed / botRadius;
    public static final double encoderRotationToMeters = 2 * Math.PI * ((wheelDiameter / 2) / gearRatio);

    // Swerve Module Base Angles
    public static final double FLZeroOffset = 217.682;

    public static final double FRZeroOffset = 227.472;

    public static final double BLZeroOffset = 200.170;

    public static final double BRZeroOffset = 311.373;

  }

  public static class Limits {
    // Elevator
    public static final double ElevMovement = 0.03;

    public static final double ElevPosThreshold = 0.5;

    // Ground Intake
    public static final double GIMinAngle = 0;

    public static final double GIMaxAngle = 180;

    public static final double GICoralDetectionCurrentThreshold = 10;

    public static final double GIAngleTolerance = 5;

    // End Effector TODO: Update these values
    public static final double EEIntakeAngleMin = 0;
    public static final double EEIntakeAngleMax = 180;
    public static final double EEIntakeCurrentThreshold = 10;
  }

  public static class Controller {
    public static final int DriverControllerPort = 0;
    public static final int SecondaryDriverControllerPort = 1;

    public static final double triggerThreshold = 0.3;
  }

  public static class Constraints {
    // Elevator
    public static final TrapezoidProfile.Constraints ElevConstraints = new TrapezoidProfile.Constraints(1, 1);

    // Ground Intake
    public static final TrapezoidProfile.Constraints GIConstraints = new TrapezoidProfile.Constraints(1, 1);
  }

  public static class Voltages {
    // Ground Intake
    public static final double GIVoltage = 8;

  }

  public static class CameraConstants {
    public static final double maxAmbiguity = 0.1;
  }

  public static class PathplannerConstants {
    public static RobotConfig config;

    // Translation PID Values
    public static final double TransP = 5.0;
    public static final double TransI = 0.0;
    public static final double TransD = 0.0;

    // Rotation PID Values
    public static final double RotP = 5.0;
    public static final double RotI = 0.0;
    public static final double RotD = 0.0;
  }

  public static class AlgaeIntakeAngles {
    public static final double stowed = 180;
    public static final double stowedAlgaeTop = 10;
    public static final double stowedAlgaeBottom = 100;
    public static final double groundIntake = 75;
    public static final double reefIntake = 30;
    public static final double processorScoreBottom = stowedAlgaeBottom;
    public static final double processorScoreTop = stowedAlgaeTop;
  }

  public static class ElevatorHeights {
    public static final double minimum = 0;
    public static final double maxiumum = 100;

    public static final double reefAlgaeL1Height = 0;
    public static final double reefAlgaeL2Height = 0;
    public static final double reefCoralL1Height = 0;
    public static final double reefCoralL2Height = 0;
    public static final double reefCoralL3Height = 0;
    public static final double reefCoralL4Height = 0;

    public static final double sourceIntake = 0;
    public static final double groundIntake = Constants.ElevatorHeights.minimum;

    public static final double processerHeight = 0;
    public static final double safeStowed = 0;
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
}