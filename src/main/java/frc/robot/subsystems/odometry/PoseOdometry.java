// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.libs.Vector2;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class PoseOdometry extends Odometry {
    protected SwerveDrivePoseEstimator estimator = null;
    private boolean knowsPosition = false;
    private Pose2d nullPose = new Pose2d(0, 0, new Rotation2d(0));

    protected PoseOdometry() {
        super();
    }

    public double getX() {
        return estimator == null ? 0 : estimator.getEstimatedPosition().getX();
    }

    public double getY() {
        return estimator == null ? 0 : estimator.getEstimatedPosition().getY();
    }

    public double getAngle() {
        return getRotation().getRadians();
    }

    public Rotation2d getRotation() {
        return estimator == null ? new Rotation2d() : estimator.getEstimatedPosition().getRotation();
    }

    public Vector2 getPosition() {
        return new Vector2(getX(), getY());
    }

    public boolean knowsPose() {
        return knowsPosition;
    }

    @Override
    public void update() {
        super.update();
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
        if (estimator != null) {
            estimator.resetPose(pose);
        }
    }

    @Override
    public Pose2d getPose() {
        return estimator == null ? nullPose : estimator.getEstimatedPosition();
    }

    public void updatePosition(SwerveModulePosition[] positions) {
        SwerveDrive drive = SwerveDrive.getInstance();
        Pose2d pose = calculatePoseFromTags();
        if (estimator == null) {
            estimator = new SwerveDrivePoseEstimator(drive.kinematics, getGyroRotation(), positions, new Pose2d());
        }
        if (pose != null) {
            if (!knowsPosition) {
                knowsPosition = true;
                estimator.resetPose(pose);
            } else {
                estimator.addVisionMeasurement(pose, Timer.getFPGATimestamp());
            }
        }

        estimator.update(getGyroRotation(), positions);
    }
}
