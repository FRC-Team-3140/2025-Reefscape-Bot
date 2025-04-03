// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.odometry;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.Vector2;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

abstract public class Odometry extends SubsystemBase {
  protected static Odometry inst = null;
  protected double lastGyroAngle;
  protected static AHRS gyro;
  private Field2d fieldEntry;

  /** Creates a new Odometry. */
  public static Odometry getInstance() {
    if (inst == null) {
      inst = new PoseOdometry();
    }
    return inst;
  }

  protected Odometry() {
    gyro = new AHRS(NavXComType.kMXP_SPI);
    resetGyro();
    lastGyroAngle = gyro.getRotation2d().getRadians() + Math.PI;
    fieldEntry = new Field2d();
    SmartDashboard.putData(fieldEntry);
  }

  protected Pose2d calculatePoseFromTags(boolean ignoreMaxDistance, boolean ignoreRepeats) {
    return Camera.getInstance().getPoseFromCamera(ignoreMaxDistance ? Integer.MAX_VALUE : Constants.CameraConstants.maxDistCutoff, ignoreRepeats);
  }

  abstract public double getX();

  abstract public double getY();

  abstract public double getAngle();

  public Rotation2d getRotation() {
    return new Rotation2d(getAngle());
  }

  abstract public Vector2 getPosition();

  abstract public boolean knowsPose();

  abstract public void recalibrateCameraPose();

  public Rotation2d getGyroRotation() {
    return new Rotation2d(gyro.getRotation2d().getRadians() + Math.PI);
  }

  public void resetGyro() {
    double delta = caluclateRotationDelta();
    gyro.reset();
    lastGyroAngle = gyro.getRotation2d().getRadians() + Math.PI - delta;
  }

  @Override
  public void periodic() {
  }

  protected double caluclateRotationDelta() {
    double delta = gyro.getRotation2d().getRadians() + Math.PI - lastGyroAngle;
    lastGyroAngle += delta;
    return delta;
  }

  public void update() {
    SwerveDrive drive = SwerveDrive.getInstance();
    SwerveModulePosition[] positions = new SwerveModulePosition[drive.modules.length];
    for (int i = 0; i < drive.modules.length; i++) {
      positions[i] = drive.modules[i].getSwerveModulePosition();
    }

    updatePosition(positions);

    fieldEntry.setRobotPose(getPose());
  }

  public void resetPose(Pose2d pose) {
    double angle = gyro.getRotation2d().getRadians() + Math.PI;
    gyro.reset();
    lastGyroAngle -= angle;
  }

  public Pose2d getPose() {
    return new Pose2d(new Translation2d(getX(), getY()), new Rotation2d(getAngle()));
  }

  public AHRS getGyro() {
    return gyro;
  }

  public boolean isMoving() {
    return getGyro().isMoving();
  }

  abstract public void updatePosition(SwerveModulePosition[] positions);
}
