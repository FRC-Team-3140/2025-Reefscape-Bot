// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.swerveDrive.SetSwerveStates;
import frc.robot.libs.FieldAprilTags;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.odometry.Odometry;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToClosestSource extends SequentialCommandGroup {
  private Odometry odometry = null;

  private Pose2d LeftSource;
  private Pose2d RightSource;

  private enum coralStations {
    LEFT,
    RIGHT
  }

  private coralStations closestStation = null;

  /** Creates a new goToClosestSource. */
  public GoToClosestSource() {
    this.odometry = Odometry.getInstance();

    // Figure out alliance and which stations to calculate from
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      LeftSource = FieldAprilTags.getInstance()
          .getTagPose(
              DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? 1 : 13);
      RightSource = FieldAprilTags.getInstance()
          .getTagPose(
              DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? 2 : 12);
    } else {
      System.err.println("Driverstation alliance wasn't present when command was called.");
      return;
    }

    Pose2d curPose = odometry.getPose();

    double leftDist = Math.pow(LeftSource.getY() - curPose.getY(), 2) + Math.pow(LeftSource.getX() - curPose.getX(), 2);
    double rightDist = Math.pow(RightSource.getY() - curPose.getY(), 2) + Math.pow(RightSource.getX() - curPose.getX(), 2);

    //closestStation = Boolean.logicalXor(leftDist < rightDist, DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)  ? coralStations.LEFT : coralStations.RIGHT;

    try {
      addCommands(
          AutoBuilder.pathfindThenFollowPath(
              (closestStation == coralStations.LEFT ? PathPlannerPath.fromPathFile("Left Source Approach")
                  : PathPlannerPath.fromPathFile("Right Source Approach")),
              Constants.PathplannerConstants.pathplannerConstraints),
          new SetSwerveStates(SwerveDrive.getInstance(), true));
      return;
    } catch (FileVersionException e) {
      System.err.print("Error in goToClosestSourse.java: \n" + e);
    } catch (IOException e) {
      System.err.print("Error in goToClosestSourse.java: \n" + e);
    } catch (ParseException e) {
      System.err.print("Error in goToClosestSourse.java: \n" + e);
    }

    addCommands(new SetSwerveStates(SwerveDrive.getInstance()));
  }
}
