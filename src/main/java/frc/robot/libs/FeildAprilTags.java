// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs;

import java.util.Hashtable;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Add your docs here. */
public class FeildAprilTags {
    private static FeildAprilTags instance = null;

    private final List<AprilTag> aprilTags;
    private final AprilTagFieldLayout field;

    private final Hashtable<Integer, AprilTag> aprilTagsHash = new Hashtable<>();

    public static FeildAprilTags getInstance() {
        if (instance == null) {
            instance = new FeildAprilTags();
        }
        return instance;
    }

    private FeildAprilTags() {
        field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        aprilTags = field.getTags();

        for (AprilTag tag : aprilTags) {
            aprilTagsHash.put(tag.ID, tag);
        }
    }

    public List<AprilTag> getTags() {
        return aprilTags;
    }

    public AprilTag getTag(int id) {
        return aprilTagsHash.get(id);
    }

    public Pose2d getTagPose(int id) {
        Optional<Pose3d> pose3dSupplier = field.getTagPose(id);
        if (!pose3dSupplier.isPresent())
            return null;
        Pose3d pose3d = pose3dSupplier.get();
        Rotation3d rot3d = pose3d.getRotation();

        return new Pose2d(pose3d.getX(), pose3d.getY(), new Rotation2d(rot3d.getAngle()));
    }

    public AprilTag getClosestAprilTag(Pose2d odometryPose) {
        AprilTag closestTag = null;
        double minDistance = Double.MAX_VALUE;

        // Iterate through each AprilTag to find the closest one
        for (AprilTag tag : aprilTags) {
            double distance = Math.sqrt(Math.pow(tag.pose.getX() - odometryPose.getX(), 2)
                    + Math.pow(tag.pose.getY() - odometryPose.getY(), 2));
            if (distance < minDistance) {
                minDistance = distance;
                closestTag = tag;
            }
        }

        return closestTag;
    }
}
