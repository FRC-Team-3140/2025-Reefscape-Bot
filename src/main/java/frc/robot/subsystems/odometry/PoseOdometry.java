// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.libs.Vector2;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class PoseOdometry extends Odometry {
    protected SwerveDrivePoseEstimator estimator = null;

    protected PoseOdometry() {
        super();
    }

    public double getX() {
        return 0;
    }

    public double getY() {
        return 0;
    }

    public double getAngle() {
        return 0;
    }

    public Rotation2d getRotation() {
        return super.getRotation();
    }

    public Vector2 getPosition() {
        return new Vector2();
    }

    public boolean knowsPose() {
        return estimator != null;
    }

    @Override
    public void update(SwerveModuleState[] states) {
        SwerveDrive drive = SwerveDrive.getInstance();
        Pose2d pose = calculatePoseFromTags();
        SwerveModulePosition[] positions = new SwerveModulePosition[drive.modules.length];
        for (int i = 0; i < drive.modules.length; i ++) {
            positions[i] = drive.modules[i].getSwerveModulePosition();
        }
        if (estimator == null && pose != null) {
            estimator = new SwerveDrivePoseEstimator(drive.kinematics, getGyroRotation(), positions, pose);
        } else if (pose != null) {
            estimator.addVisionMeasurement(pose, Timer.getFPGATimestamp());
        }

        if (estimator != null) {
          estimator.update(getGyroRotation(), positions);
        }
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public void updatePosition(SwerveModuleState[] states) {
    }
}
