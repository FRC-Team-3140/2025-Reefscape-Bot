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
import frc.robot.Constants;
import frc.robot.libs.FieldAprilTags;
import frc.robot.libs.LoggedCommand;
import frc.robot.subsystems.odometry.Odometry;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToClosestSource extends LoggedCommand {
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

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(odometry);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();

    // Figure out alliance and which stations to calculate from
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      LeftSource = FieldAprilTags.getInstance()
          .getTagPose(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 2 : 13);
      RightSource = FieldAprilTags.getInstance()
          .getTagPose(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 1 : 12);
    } else {
      System.err.println("Driverstation alliance wasn't present when command was called.");
      end(true);
    }

    Pose2d curPose = odometry.getPose();

    double leftDist = Math
        .sqrt(Math.pow(LeftSource.getY() - curPose.getY(), 2) + Math.pow(LeftSource.getX() - curPose.getX(), 2));
    double rightDist = Math
        .sqrt(Math.pow(RightSource.getY() - curPose.getY(), 2) + Math.pow(RightSource.getX() - curPose.getX(), 2));

    double minDist = Math.min(leftDist, rightDist);
    closestStation = (minDist == leftDist) ? coralStations.LEFT : coralStations.RIGHT;

    try {
      AutoBuilder
          .pathfindThenFollowPath(
              (closestStation == coralStations.LEFT ? PathPlannerPath.fromPathFile("Left Source Approach")
                  : PathPlannerPath.fromPathFile("Right Source Approach")),
              Constants.PathplannerConstants.pathplannerConstraints)
          .schedule();
    } catch (FileVersionException e) {
      System.err.print("Error in goToClosestSourse.java: \n" + e);
      end(true);
    } catch (IOException e) {
      System.err.print("Error in goToClosestSourse.java: \n" + e);
      end(true);
    } catch (ParseException e) {
      System.err.print("Error in goToClosestSourse.java: \n" + e);
      end(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Should end immedately since another command is scheduled in init.
    return true;
  }
}
