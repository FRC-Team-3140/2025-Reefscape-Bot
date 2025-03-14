// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compoundCommands;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeights;
import frc.robot.commands.elevator.SetHeight;
import frc.robot.commands.swerveDrive.SetSwerveStates;
import frc.robot.libs.NetworkTables;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionAndScoreCoral extends SequentialCommandGroup {
  private Elevator elevator = null;

  private Pose2d finalPose = null;

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

    boolean allianceBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

    Constants.ReefPoses reefPositions = new Constants.ReefPoses();

    HashMap<Integer, Pose2d> reefPoses = null;

    if (allianceBlue) {
      reefPoses = reefPositions.reefCoralPosesBlue;
    } else {
      reefPoses = reefPositions.reefCoralPosesRed;
    }

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

    switch (posint) {
      case -1:
        // Right = even
        switch (reefSide) {
          case 0:
            finalPose = reefPoses.get(0);
            break;

          case 1:
            finalPose = reefPoses.get(2);
            break;

          case 2:
            finalPose = reefPoses.get(4);
            break;

          case 3:
            finalPose = reefPoses.get(6);
            break;

          case 4:
            finalPose = reefPoses.get(8);
            break;

          case 5:
            finalPose = reefPoses.get(10);
            break;

          default:
            end(true);
            break;
        }
        break;

      case 1:
        // Left = odd
        switch (reefSide) {
          case 0:
            finalPose = reefPoses.get(1);
            break;

          case 1:
            finalPose = reefPoses.get(3);
            break;

          case 2:
            finalPose = reefPoses.get(5);
            break;

          case 3:
            finalPose = reefPoses.get(7);
            break;

          case 4:
            finalPose = reefPoses.get(9);
            break;

          case 5:
            finalPose = reefPoses.get(11);
            break;

          default:
            end(true);
            break;
        }
        break;

      default:
        end(true);
        break;
    }

    System.out.println(finalPose.getX() + " " + finalPose.getY() + " " + finalPose.getRotation());

    // Put the edge of the bot theoretically touching the apriltag
    pathfindingCommand = AutoBuilder.pathfindToPose(
        finalPose,
        Constants.PathplannerConstants.pathplannerConstraints, 0.0);

    // Schedule the pathfinding command to run along with this command that will
    // handle the elevator
    addCommands(new InstantCommand(() -> NetworkTables.cameraPose
        .setDoubleArray(new double[] {
            finalPose.getX(),
            finalPose.getY(),
            finalPose.getRotation().getDegrees() })),
        pathfindingCommand,
        new SetSwerveStates(SwerveDrive.getInstance(), true),
        new SetHeight(level));
  }
}