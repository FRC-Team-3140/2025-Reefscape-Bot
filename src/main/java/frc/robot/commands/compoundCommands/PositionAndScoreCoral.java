// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compoundCommands;

import java.util.Hashtable;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeights;
import frc.robot.commands.swerveDrive.setSwerveStates;
import frc.robot.libs.NetworkTables;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.odometry.Odometry;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionAndScoreCoral extends SequentialCommandGroup {
  private Elevator elevator = null;
  private Odometry odometry = null;

  private Pose2d finalPose = null;
  private double minDistForElevator = 0.5;

  private Hashtable<Integer, Pose2d> reefPoses = new Hashtable<>();

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
    this.odometry = Odometry.getInstance();

    coralScorePos = pos;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, odometry);

    reefPoses.put(0, new Pose2d(3.915, 3.85, new Rotation2d(Units.degreesToRadians(0))));
    reefPoses.put(1, new Pose2d(3.915, 4.18, new Rotation2d(Units.degreesToRadians(0))));
    reefPoses.put(2, new Pose2d(3.683, 5.062, new Rotation2d(Units.degreesToRadians(-60))));
    reefPoses.put(3, new Pose2d(3.978, 5.245, new Rotation2d(Units.degreesToRadians(-60))));
    reefPoses.put(4, new Pose2d(4.994, 5.235, new Rotation2d(Units.degreesToRadians(-120))));
    reefPoses.put(5, new Pose2d(5.299, 5.082, new Rotation2d(Units.degreesToRadians(-120))));
    reefPoses.put(6, new Pose2d(5.7787, 4.18, new Rotation2d(Units.degreesToRadians(180))));
    reefPoses.put(7, new Pose2d(5.7787, 3.85, new Rotation2d(Units.degreesToRadians(180))));
    reefPoses.put(8, new Pose2d(5.289, 2.988, new Rotation2d(Units.degreesToRadians(120))));
    reefPoses.put(9, new Pose2d(5, 2.805, new Rotation2d(Units.degreesToRadians(120))));
    reefPoses.put(10, new Pose2d(3.967, 2.815, new Rotation2d(Units.degreesToRadians(60))));
    reefPoses.put(11, new Pose2d(3.693, 2.978, new Rotation2d(Units.degreesToRadians(60))));

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

    // TODO: Will have to duplicate the reefPoses for the red alliance and get the
    // measurements
    // boolean allianceBlue = DriverStation.getAlliance().get() ==
    // DriverStation.Alliance.Blue;

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
        pathfindingCommand, new setSwerveStates(SwerveDrive.getInstance(), Constants.Bot.defaultSwerveStates),
        new elevatorCommand());
  }

  private class elevatorCommand extends Command {
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
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      if (interrupted)
        System.err.println("Interrupted or Issues encountered while running ScoreCoral command.");

      elevator.setHeight(level);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      Pose2d curPose = odometry.getPose();
      double distance = Math
          .sqrt(Math.pow(finalPose.getY() - curPose.getY(), 2) + Math.pow(finalPose.getX() - curPose.getX(), 2));

      if (distance <= minDistForElevator) {
        return true;
      } else {
        return false;
      }
    }
  }
}