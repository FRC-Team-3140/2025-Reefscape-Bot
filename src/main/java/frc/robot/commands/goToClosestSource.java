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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.odometry.Odometry;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class goToClosestSource extends Command {
  private Odometry odometry = null;

  private Pose2d LeftSource = new Pose2d(1, 14, new Rotation2d());
  private Pose2d RightSource = new Pose2d(1, 1, new Rotation2d());

  private enum coralStations {
    LEFT,
    RIGHT
  }

  private coralStations closestStation = null;

  /** Creates a new goToClosestSource. */
  public goToClosestSource(Odometry odometry) {
    this.odometry = odometry;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(odometry);
  }

  private double sqr(double num) {
    return (num * num);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d curPose = odometry.getPose();

    double leftDist = Math.sqrt(sqr(LeftSource.getY() - curPose.getY()) + sqr(LeftSource.getX() - curPose.getX()));
    double rightDist = Math.sqrt(sqr(RightSource.getY() - curPose.getY()) + sqr(RightSource.getX() - curPose.getX()));

    double minDist = Math.min(leftDist, rightDist);
    closestStation = (minDist == leftDist) ? coralStations.LEFT : coralStations.RIGHT;

    try {
      AutoBuilder
          .pathfindThenFollowPath(
              (closestStation == coralStations.LEFT ? PathPlannerPath.fromPathFile("Left Source Approach")
                  : PathPlannerPath.fromPathFile("Right Source Approach")),
              Constants.Constraints.pathplannerConstraints)
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Should end immedately since another command is scheduled in init. 
    return true;
  }
}
