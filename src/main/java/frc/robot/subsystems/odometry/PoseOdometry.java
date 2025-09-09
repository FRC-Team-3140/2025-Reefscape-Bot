// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.libs.Vector2;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class PoseOdometry extends Odometry {
    protected SwerveDrivePoseEstimator estimator = null;
    private boolean knowsPosition = false;
    private Pose2d nullPose = new Pose2d(0, 0, new Rotation2d(0));

    private Pose2d startingPose = null;
    private final int startingCameraPasses = 10;
    private int cameraPasses = 0;
    private double angleOffset = 0;

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
        if (estimator == null) {
            estimator = new SwerveDrivePoseEstimator(
                    SwerveDrive.getInstance().kinematics,
                    pose.getRotation(),
                    SwerveDrive.getInstance().getModulePositions(),
                    pose);
        } else {
            estimator.resetPose(pose);
        }

        if (RobotBase.isSimulation()) {
            NavXSim.getInstance().reset(pose.getRotation().getRadians());

            // Reset all the module encoders to 0
            for (SwerveModule module : SwerveDrive.getInstance().modules) {
                module.simDriveEncoder.setPosition(0);
                module.simDriveMotor.setPosition(0);

                module.simTurnEncoder.setPosition(0);
                module.simTurnMotor.setPosition(0);
            }

            updateSimulatedPosition(SwerveDrive.getInstance().getModulePositions(),
                    NavXSim.getInstance().getRotation2d().getRadians());
        }

        System.out.println("[Odometry] Reset Pose to " + pose);
    }

    @Override
    public Pose2d getPose() {
        return estimator == null ? nullPose : estimator.getEstimatedPosition();
    }

    @Override
    public Rotation2d getGyroRotation() {
        if (!RobotBase.isSimulation())
            return new Rotation2d(gyro.getRotation2d().getRadians() + angleOffset);
        else
            return NavXSim.getInstance().getRotation2d();
    }

    public void resetGyro() {
        if (!RobotBase.isSimulation())
            resetGyroCamera(0);
        else
            NavXSim.getInstance().reset(estimator.getEstimatedPosition().getRotation().getRadians());
    }

    public void resetGyroCamera(double correctAngle) {
        if (!RobotBase.isSimulation())
            angleOffset = -readRotationRaw() + correctAngle;
    }

    public void recalibrateCameraPose() {
        cameraPasses = 0;
    }

    @Override
    public void updatePosition(SwerveModulePosition[] positions) {
        if (RobotBase.isSimulation()) {
            updateSimulatedPosition(positions, NavXSim.getInstance().getRotation2d().getRadians());
            return;
        }

        SwerveDrive drive = SwerveDrive.getInstance();
        Pose2d poseClipped = calculatePoseFromTags(false, false);
        Pose2d pose = calculatePoseFromTags(true, true);
        if (pose == null)
            pose = poseClipped;

        if (estimator == null) {
            estimator = new SwerveDrivePoseEstimator(
                    drive.kinematics,
                    getGyroRotation(),
                    positions,
                    new Pose2d());
            estimator.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, Units.degreesToRadians(15)));
        }

        if (cameraPasses == 0) {
            startingPose = null;
            cameraPasses++;
        } else if (cameraPasses < startingCameraPasses) {
            if (pose != null) {
                if (startingPose == null)
                    startingPose = pose;
                startingPose = startingPose.interpolate(pose, 1.0 / startingCameraPasses);
                cameraPasses++;
            }
        } else if (cameraPasses == startingCameraPasses) {
            estimator.resetPose(startingPose);
            resetGyroCamera(startingPose.getRotation().getRadians());
            cameraPasses++;
        } else {
            if (poseClipped != null) {
                if (!knowsPosition) {
                    knowsPosition = true;
                    estimator.resetPose(poseClipped);
                } else {
                    if (estimator.getEstimatedPosition().getTranslation()
                            .getDistance(poseClipped.getTranslation()) < 0.5) {
                        estimator.addVisionMeasurement(
                                new Pose2d(poseClipped.getX(), poseClipped.getY(), getGyroRotation()),
                                Timer.getFPGATimestamp());
                    }
                }
            }
        }

        estimator.update(getGyroRotation(), positions);
    }

    @Override
    public void updateSimulatedPosition(SwerveModulePosition[] positions, double gyroAngleRad) {
        if (estimator == null) {
            estimator = new SwerveDrivePoseEstimator(
                    SwerveDrive.getInstance().kinematics,
                    new Rotation2d(gyroAngleRad),
                    positions,
                    new Pose2d());
        }
        estimator.update(new Rotation2d(gyroAngleRad), positions);
    }
}
