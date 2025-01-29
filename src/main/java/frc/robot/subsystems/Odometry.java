// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.Vector2;

public class Odometry extends SubsystemBase {
  private Vector2 position = null;
  private Double angle = null; // IN RADIANS
  private double lastUpdate = Timer.getFPGATimestamp();
  private static Odometry inst = null;
  private double lastGyroAngle;
  public static AHRS gyro;


  /** Creates a new Odometry. */
  public static Odometry getInstance() {
    if (inst == null) {
      inst = new Odometry();
    }
    return inst;
  }

  private Odometry() {
    gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro.reset();
    lastGyroAngle = gyro.getRotation2d().getRadians();
  }

  public double GetX() {return position == null ? 0 : position.X;}
  public double GetY() {return position == null ? 0 : position.Y;}
  public double getAngle() {
    if (angle == null) {
      return 0;
    }
    return angle % (Math.PI * 2);
  }
  public Rotation2d getRotation() {return new Rotation2d(getAngle());}
  public Vector2 GetPosition() {return position == null ? new Vector2() : position;}
  public boolean KnowsPose() {return position != null && angle != null;}

  private Pose2d CalculatePoseFromTags() {
    

    return null;
  }

  private Vector2 CalculateEncoderDelta() {
    return new Vector2(0d, 0d);
  }

  private double CaluclateRotationDelta() {
    double delta = gyro.getRotation2d().getRadians() - lastGyroAngle;
    lastGyroAngle += delta;
    return delta;
  }

  @Override
  public void periodic() {
    updatePosition();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    StructPublisher<Pose2d> odometryStruct = inst.getStructTopic("Odometry", Pose2d.struct).publish();
    odometryStruct.set(getPose());
  }

  public void resetPose(Pose2d pose) {
    position = new Vector2(pose.getX(), pose.getY());
    angle = pose.getRotation().getRadians();
    double angle = gyro.getRotation2d().getRadians();
    gyro.reset();
    lastGyroAngle -= angle;
  }

  public Pose2d getPose() {
    return new Pose2d(new Translation2d(GetX(), GetY()), new Rotation2d(getAngle()));
  }

  public void updatePosition() {
    Pose2d tagPose = CalculatePoseFromTags();
    double deltaTime = Timer.getFPGATimestamp() - lastUpdate;
    lastUpdate += deltaTime;

    // if it found a camera position and it doesnt have a better position, use that
    if (tagPose != null && !KnowsPose()) {
      position = new Vector2(tagPose.getX(), tagPose.getY());
      angle = tagPose.getRotation().getRadians();
      return;
    }

    // if it still doesnt have a position, we cant use the delta, so just stop
    if (!KnowsPose())
      return;
    
    // move the position based on the delta calculated from the encoders
    Vector2 delta = CalculateEncoderDelta();
    position = position.add(delta);
    double rotation = CaluclateRotationDelta();
    angle += rotation;

    // weightedly move the calculated position toward what the tags are saying.
    if (tagPose != null) {
      Vector2 tagPos = new Vector2(tagPose.getX(), tagPose.getY());

      double alpha = 1 - Math.exp(-deltaTime*Constants.Odometry.TagCorrectionSpeed);

      position = position.lerp(tagPos, alpha);

      double targetAngle = tagPose.getRotation().getRadians();
      while (Math.abs(targetAngle - angle) > Math.PI) {
        if (targetAngle > angle) {
          angle += Math.PI * 2;
        } else {
          angle -= Math.PI * 2;
        }
      }
      angle = (targetAngle - angle)*alpha + angle;
    } 
  }
}
