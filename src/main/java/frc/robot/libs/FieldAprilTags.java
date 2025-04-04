// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs;

import java.util.ArrayList;
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
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class FieldAprilTags {
    private static FieldAprilTags instance = null;

    private final List<AprilTag> aprilTags;
    private List<AprilTag> reefTags = new ArrayList<>();
    private final int[] reefIDs = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
    public final AprilTagFieldLayout field;

    private final Hashtable<Integer, AprilTag> aprilTagsHash = new Hashtable<>();

    public final Hashtable<Integer, AprilTag> blueReefTagsHash = new Hashtable<>();
    public final Hashtable<Integer, AprilTag> redReefTagsHash = new Hashtable<>();

    /**
     * @param aprilTag
     * @param reefSide
     */
    public class ReefAprilTag {
        public final AprilTag aprilTag;
        public final int reefSide;

        public ReefAprilTag(AprilTag aprilTag, int reefSide) {
            this.aprilTag = aprilTag;
            this.reefSide = reefSide;
        }
    }

    public static FieldAprilTags getInstance() {
        if (instance == null) {
            instance = new FieldAprilTags();
        }
        return instance;
    }

    private FieldAprilTags() {
        field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        aprilTags = field.getTags();

        for (AprilTag tag : aprilTags) {
            aprilTagsHash.put(tag.ID, tag);
        }

        for (int reefID : reefIDs) {
            reefTags.add(aprilTagsHash.get(reefID));
        }

        // Side 0 is the side facing the driverstation and continues clockwise
        blueReefTagsHash.put(0, aprilTagsHash.get(18));
        blueReefTagsHash.put(1, aprilTagsHash.get(19));
        blueReefTagsHash.put(2, aprilTagsHash.get(20));
        blueReefTagsHash.put(3, aprilTagsHash.get(21));
        blueReefTagsHash.put(4, aprilTagsHash.get(22));
        blueReefTagsHash.put(5, aprilTagsHash.get(17));

        redReefTagsHash.put(0, aprilTagsHash.get(7));
        redReefTagsHash.put(1, aprilTagsHash.get(6));
        redReefTagsHash.put(2, aprilTagsHash.get(11));
        redReefTagsHash.put(3, aprilTagsHash.get(10));
        redReefTagsHash.put(4, aprilTagsHash.get(9));
        redReefTagsHash.put(5, aprilTagsHash.get(8));
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

    /**
     * Returns an Apriltag and a Alliance relative reef side 0-5
     * 
     * @param odometryPose
     * @return
     */
    public ReefAprilTag getClosestReefAprilTag(Pose2d odometryPose, DriverStation.Alliance alliance) {
        AprilTag closestTag = null;
        double minDistance = Double.MAX_VALUE;

        Hashtable<Integer, AprilTag> closestHash = (alliance == DriverStation.Alliance.Blue ? blueReefTagsHash
                : redReefTagsHash);

        // Iterate through each AprilTag to find the closest one
        for (AprilTag tag : closestHash.values()) {
            double distance = Math.sqrt(Math.pow(tag.pose.getX() - odometryPose.getX(), 2)
                    + Math.pow(tag.pose.getY() - odometryPose.getY(), 2));
            if (distance < minDistance) {
                minDistance = distance;
                closestTag = tag;
            }
        }

        int reefSide = -1;

        for (int key : closestHash.keySet()) {
            if (closestTag.equals(closestHash.get(key))) {
                reefSide = key;
                break;
            }
        }

        return new ReefAprilTag(closestTag, reefSide);
    }
}
