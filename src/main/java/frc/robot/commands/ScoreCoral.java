// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeights;
import frc.robot.libs.FeildAprilTags;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.odometry.Odometry;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {
  private Elevator elevator = null;

  public enum Position {
    L_1,
    L_2,
    L_3,
    L_4,
    R_1,
    R_2,
    R_3,
    R_4
  }

  private Position coralScorePos = null;

  private PathConstraints constraints = new PathConstraints(
      Constants.Bot.maxChassisSpeed,
      4.0,
      Units.degreesToRadians(540),
      Units.degreesToRadians(720));

  private Command pathfindingCommand = null;

  private Double level = null;

  /** Creates a new ScoreCoral. */
  public ScoreCoral(Elevator elevator, Camera camera, Odometry odometry, Position pos) {
    this.elevator = elevator;

    coralScorePos = pos;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, camera, odometry);

    String[] posParts = coralScorePos.name().split("_");
    String side = posParts[0];
    String position = posParts[1];
    System.out.println("Side: " + side + ", Position: " + position);

    int posint = 0;

    switch (side) {
      case "L":
        posint = 1;
        break;
      case "R":
        posint = -1;
        break;
      default:
        System.err.println("Somehow magically passed in invalid position...");
        end(true);
        break;
    }

    switch (Integer.parseInt(position)) {
      case 1:
        level = ElevatorHeights.reefCoralL1Height;
        break;

      case 2:
        level = ElevatorHeights.reefCoralL2Height;
        break;

      case 3:
        level = ElevatorHeights.reefCoralL3Height;
        break;

      case 4:
        level = ElevatorHeights.reefCoralL4Height;
        break;

      default:
        System.err.println("Somehow magically passed in invalid position...");
        end(true);
        break;
    }

    // Figure out where to position robot on reef
    AprilTag closestTag = FeildAprilTags.getInstance().getClosestAprilTag(odometry.getPose());
    Pose3d pose = closestTag.pose;

    System.out.println("Cloest Tag ID: " + closestTag.ID);
    System.out.println("Closets Tag Pose: \n" + "x:" + pose.getX() + "\n" + "y:"
        + pose.getY() + "\n" + "yaw:" + pose.getRotation().getAngle());
        
    double angle = pose.getRotation().getAngle() + posint * (Math.PI / 4);
    double distance = (Constants.Bot.botLength / 2)/Math.tan(angle);

    // Put the edge of the bot theoretically touching the apriltag
    pathfindingCommand = AutoBuilder.pathfindToPose(
        new Pose2d(
            pose.getX() + distance * Math.cos(angle),
            pose.getY() + distance * Math.sin(angle),
            new Rotation2d(Units.degreesToRadians(pose.getRotation().getAngle()) + Math.PI / 2)),
        constraints);

    // Schedule the pathfinding command to run along with this command that will
    // handle the elevator
    this.alongWith(pathfindingCommand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (level == null) {
      System.err.println("Error: Level is null. Exiting command.");
      end(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setHeight(level);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      System.err.println("Interrupted or Issues encountered while running ScoreCoral command.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}