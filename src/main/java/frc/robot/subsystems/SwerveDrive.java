// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  }

  public void periodic() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      SwerveModule module = modules[i];
      states[i] = module.getState();
    }
    odometry.update(states);
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
      modules[i].setStates(swerveModuleStates[i], false);
    }
  }

  public boolean shouldFlipPath() {
    return DriverStation.getAlliance().get().equals(Alliance.Red);
  }

  // public Pose2d getExpectedPose() {
  // return getPose().plus(new Transform2d(
  // new Translation2d(botSpeeds.vxMetersPerSecond * .05,
  // botSpeeds.vyMetersPerSecond * .05), new Rotation2d()));
  // }
}