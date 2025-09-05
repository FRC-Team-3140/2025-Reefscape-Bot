// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.libs.Vector2;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CustomOdometry extends Odometry {
    private Vector2 position = new Vector2(); // THIS AND ANGLE USED TO BE NULLABLE, NOT ANYMORE BUT KEPT COMPATIBILITY
    private Double angle = Double.valueOf(0); // IN RADIANS
    private SwerveModuleState[] lastStates = null;
    private Double lastStatesT = null;
    private Double lastUpdateT = null;
    private boolean knowsPosition = false;

    protected CustomOdometry() {
        super();
    }

    public double getX() {
        return position == null ? 0 : position.X;
    }

    public double getY() {
        return position == null ? 0 : position.Y;
    }

    public double getAngle() {
        if (angle == null) {
            return 0;
        }
        double a = angle % (Math.PI * 2);
        if (a < 0) {
            a += Math.PI * 2;
        }
        return a;
    }

    public Rotation2d getRotation() {
        return new Rotation2d(getAngle());
    }

    public Vector2 getPosition() {
        return position == null ? new Vector2() : position;
    }

    public boolean knowsPose() {
        return position != null && angle != null;
    }

    public Rotation2d getGyroRotation() {
        return gyro.getRotation2d();
    }

    public void recalibrateCameraPose() {
    }

    private double optimizeAngle(double baseline, double angle) {
        while (Math.abs(angle - baseline) > Math.PI) {
            if (baseline > angle) {
                angle += Math.PI * 2;
            } else {
                angle -= Math.PI * 2;
            }
        }
        return angle;
    }

    private Vector2 calculateEncoderDelta(SwerveModuleState[] states) {
        if (lastStates == null || lastStatesT == null) {
            lastStates = states;
            lastStatesT = Timer.getFPGATimestamp();
            return new Vector2();
        }

        Vector2 delta = new Vector2();
        double tc = Timer.getFPGATimestamp() - lastStatesT;
        lastStatesT += tc;

        for (int i = 0; i < states.length; i++) {
            double a0 = lastStates[i].angle.getRadians() + getAngle();
            double af = optimizeAngle(a0, states[i].angle.getRadians() + getAngle());
            double da = af - a0;

            double v0 = lastStates[i].speedMetersPerSecond;
            double vf = states[i].speedMetersPerSecond;
            double dv = vf - v0;

            //////////// UNFANCY CALCULUS FREE BASIC CODE (less accurate)
            // double angle = states[i].angle.getRadians() + getAngle();
            // Vector2 dir = new Vector2(Math.cos(angle), Math.sin(angle));
            // delta = delta.add(dir.mult(states[i].speedMetersPerSecond * deltaTime));

            //////////// FANCY CALCULUS BASED COMPLEX CODE (accurate (probably))
            delta = delta.add(new Vector2(
                    da == 0 ? (vf - v0) / 2 * tc * Math.cos(a0)
                            : tc * (vf / da * Math.sin(af) - v0 / da * Math.sin(a0)
                                    + dv / (da * da) * (Math.cos(af) - Math.cos(a0))),
                    da == 0 ? (vf - v0) / 2 * tc * Math.sin(a0)
                            : tc * (-vf / da * Math.cos(af) + v0 / da * Math.cos(a0)
                                    + dv / (da * da) * (Math.sin(af) - Math.sin(a0)))));
        }

        lastStates = states;

        delta = delta.div(states.length);

        return delta;
    }

    public void update() {
        super.update();
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
        position = new Vector2(pose.getX(), pose.getY());
        angle = pose.getRotation().getRadians();
    }

    @Override
    public Pose2d getPose() {
        return super.getPose();
    }

    public void resetGyro() {
        double delta = caluclateRotationDelta();
        gyro.reset();
        lastGyroAngle = gyro.getRotation2d().getRadians() - delta;
    }

    public void updatePosition(SwerveModulePosition[] positions) {
        if (lastUpdateT == null) {
            lastUpdateT = Timer.getFPGATimestamp();
        }

        double deltaTime = Timer.getFPGATimestamp() - lastUpdateT;
        lastUpdateT += deltaTime;
        Pose2d tagPose = calculatePoseFromTags(false, false);

        // if it found a camera position and it doesnt have a better position, use that
        if (tagPose != null && !knowsPosition) {
            knowsPosition = true;
            position = new Vector2(tagPose.getX(), tagPose.getY());
            angle = tagPose.getRotation().getRadians();
        }

        // incase these values are null, we can't do anything
        if (position == null || angle == null) {
            return;
        }

        // weightedly move the calculated position toward what the tags are saying.
        if (tagPose != null) {
            Vector2 tagPos = new Vector2(tagPose.getX(), tagPose.getY());

            double alpha = 1 - Math.exp(-deltaTime * Constants.Odometry.TagCorrectionSpeed);

            position = position.lerp(tagPos, alpha);

            double targetAngle = tagPose.getRotation().getRadians();
            angle = optimizeAngle(targetAngle, angle);
            angle = (targetAngle - angle) * alpha + angle;
        }

        // move the position based on the delta calculated from the encoders
        double rotation = caluclateRotationDelta();
        angle += rotation;

        SwerveDrive drive = SwerveDrive.getInstance();
        SwerveModuleState[] states = new SwerveModuleState[drive.modules.length];
        for (int i = 0; i < drive.modules.length; i++) {
            states[i] = drive.modules[i].getState();
        }
        Vector2 delta = calculateEncoderDelta(states);
        position = position.add(delta);
    }

    @Override
    public void updateSimulatedPosition(SwerveModulePosition[] positions, double gyroAngleRad) {
        // TODO: I'm too lazy to figure out how this works right now :) - TK
    }
}
