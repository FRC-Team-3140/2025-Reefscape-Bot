// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compoundCommands;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeights;
import frc.robot.commands.elevator.SetHeight;
import frc.robot.commands.swerveDrive.Align;
import frc.robot.libs.FieldAprilTags;
import frc.robot.libs.LoggedCommand;
import frc.robot.subsystems.odometry.Odometry;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionFromDash extends LoggedCommand {
  private Double level = null;
  private Pose2d finalPose = null;
  private Command pathfindingCommand = null;

  /**
   * Creates a new ScoreCoral.
   * 
   * @param Elevator
   * @param Odometry
   * @param Position
   * @param reefSide
   */
  public PositionFromDash(String pos, boolean algae) {
    boolean allianceBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

    Constants.ReefPoses reefPositions = new Constants.ReefPoses();

    HashMap<Integer, Pose2d> reefPoses = null;

    if (allianceBlue) {
      reefPoses = algae ? reefPositions.reefAlgaePosesBlue : reefPositions.reefCoralPosesBlue;
    } else {
      reefPoses = algae ? reefPositions.reefAlgaePosesRed : reefPositions.reefCoralPosesRed;
    }

    String[] posParts = pos.split("_");
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

    if (algae) {
      switch (Integer.parseInt(position)) {
        case 1:
          level = ElevatorHeights.reefAlgaeL1Height;
          break;

        case 2:
          level = ElevatorHeights.reefAlgaeL1Height;
          break;

        case 3:
          level = ElevatorHeights.reefAlgaeL2Height;
          break;

        case 4:
          level = ElevatorHeights.reefAlgaeL2Height;
          break;

        default:
          System.err.println("Somehow magically passed in invalid position...");
          end(true);
          break;
      }
    } else {
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
    }

    int reefSide = FieldAprilTags.getInstance().getClosestReefAprilTag(
        Odometry.getInstance().getPose(),
        DriverStation.getAlliance().get()).reefSide;

    switch (posint) {
      case -1:
        // Right = even
        switch (reefSide) {
          case 0:
            if (algae) {
              finalPose = reefPoses.get(0);
            } else {
              finalPose = reefPoses.get(0);
            }
            break;

          case 1:
            if (algae) {
              finalPose = reefPoses.get(1);
            } else {
              finalPose = reefPoses.get(2);
            }
            break;

          case 2:
            if (algae) {
              finalPose = reefPoses.get(2);
            } else {
              finalPose = reefPoses.get(4);
            }
            break;

          case 3:
            if (algae) {
              finalPose = reefPoses.get(3);
            } else {
              finalPose = reefPoses.get(6);
            }
            break;

          case 4:
            if (algae) {
              finalPose = reefPoses.get(4);
            } else {
              finalPose = reefPoses.get(8);
            }
            break;

          case 5:
            if (algae) {
              finalPose = reefPoses.get(5);
            } else {
              finalPose = reefPoses.get(10);
            }
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
            if (algae) {
              finalPose = reefPoses.get(0);
            } else {
              finalPose = reefPoses.get(1);
            }
            break;

          case 1:
            if (algae) {
              finalPose = reefPoses.get(1);
            } else {
              finalPose = reefPoses.get(3);
            }
            break;

          case 2:
            if (algae) {
              finalPose = reefPoses.get(2);
            } else {
              finalPose = reefPoses.get(5);
            }
            break;

          case 3:
            if (algae) {
              finalPose = reefPoses.get(3);
            } else {
              finalPose = reefPoses.get(7);
            }
            break;

          case 4:
            if (algae) {
              finalPose = reefPoses.get(4);
            } else {
              finalPose = reefPoses.get(9);
            }
            break;

          case 5:
            if (algae) {
              finalPose = reefPoses.get(5);
            } else {
              finalPose = reefPoses.get(11);
            }
            break;

          default:
            end(true);
            break;
        }

        System.out.println(finalPose.getX() + " " + finalPose.getY() + " " + finalPose.getRotation());

        // Put the edge of the bot theoretically touching the apriltag
        System.out.println("Generating..");
        pathfindingCommand = AutoBuilder.pathfindToPose(
            finalPose,
            Constants.PathplannerConstants.pathplannerConstraints, 0.0).andThen(new Align(finalPose));
        System.out.println("Done");
        System.out.println("Running...");
        pathfindingCommand.schedule();
        

    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    new SetHeight(level).schedule();
  }

  public boolean isFinished() {
    return pathfindingCommand != null ? pathfindingCommand.isFinished() : false;
  }
}