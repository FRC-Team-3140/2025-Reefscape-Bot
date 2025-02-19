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
    private Vector2 position = null;
    private Double angle = null; // IN RADIANS

    private CustomOdometry() {
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

    private Vector2 calculateEncoderDelta(SwerveModuleState[] states, double deltaTime) {
        Vector2 delta = new Vector2();

        for (int i = 0; i < states.length; i++) {
            double angle = states[i].angle.getRadians() + getAngle();
            Vector2 dir = new Vector2(Math.cos(angle), Math.sin(angle));
            delta = delta.add(dir.mult(states[i].speedMetersPerSecond * deltaTime));
        }

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

    public void updatePosition(SwerveModulePosition[] positions) {
        Pose2d tagPose = calculatePoseFromTags();
        double deltaTime = Timer.getFPGATimestamp() - lastUpdate;
        lastUpdate += deltaTime;

        // if it found a camera position and it doesnt have a better position, use that
        if (tagPose != null && !knowsPose()) {
            position = new Vector2(tagPose.getX(), tagPose.getY());
            angle = tagPose.getRotation().getRadians();
            return;
        }

        // if it still doesnt have a position, we cant use the delta, so just stop
        if (!knowsPose())
            return;

        // move the position based on the delta calculated from the encoders
        double rotation = caluclateRotationDelta();
        angle += rotation;

        SwerveDrive drive = SwerveDrive.getInstance();
        SwerveModuleState[] states = new SwerveModuleState[drive.modules.length];
        for (int i = 0; i < drive.modules.length; i ++) {
            states[i] = drive.modules[i].getState();
        }
        Vector2 delta = calculateEncoderDelta(states, deltaTime);
        position = position.add(delta);

        // weightedly move the calculated position toward what the tags are saying.
        if (tagPose != null) {
            Vector2 tagPos = new Vector2(tagPose.getX(), tagPose.getY());

            double alpha = 1 - Math.exp(-deltaTime * Constants.Odometry.TagCorrectionSpeed);

            position = position.lerp(tagPos, alpha);

            double targetAngle = tagPose.getRotation().getRadians();
            while (Math.abs(targetAngle - angle) > Math.PI) {
                if (targetAngle > angle) {
                    angle += Math.PI * 2;
                } else {
                    angle -= Math.PI * 2;
                }
            }
            angle = (targetAngle - angle) * alpha + angle;
        }
    }
}
