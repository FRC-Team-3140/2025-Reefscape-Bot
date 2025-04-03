// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compoundCommands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeights;
import frc.robot.commands.elevator.SetHeight;
import frc.robot.commands.swerveDrive.SetSwerveStates;
import frc.robot.libs.FieldAprilTags;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.odometry.Odometry;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionAndScoreCoral extends SequentialCommandGroup {
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

  private Command pathfindingCommand = null;

  private Double level = null;

  /**
   * Creates a new ScoreCoral.
   * 
   * @param Elevator
   * @param Odometry
   * @param Position
   * @param reefSide
   */
  public PositionAndScoreCoral(Position pos, int reefSide) {
    this.elevator = Elevator.getInstance();

    coralScorePos = pos;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    Pose2d reefPose = null;

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

    reefPose = Constants.ReefPoses.getPose(
        // If reef side is -1 it will go to closest.
        reefSide == -1
            ? FieldAprilTags.getInstance().getClosestReefAprilTag(Odometry.getInstance().getPose(),
                DriverStation.getAlliance().get()).reefSide
            : reefSide,
        posint);

    System.out.println(reefPose.getX() + " " + reefPose.getY() + " " + reefPose.getRotation());

    // Put the edge of the bot theoretically touching the apriltag
    pathfindingCommand = AutoBuilder.pathfindToPose(
        reefPose,
        Constants.PathplannerConstants.pathplannerConstraints, 0.0);

    // Schedule the pathfinding command to run along with this command that will
    // handle the elevator
    addCommands(
        pathfindingCommand,
        new SetSwerveStates(SwerveDrive.getInstance(), true),
        new SetHeight(level));
  }
}