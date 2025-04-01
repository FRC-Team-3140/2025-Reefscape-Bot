// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compoundCommands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.elevator.SetHeight;
import frc.robot.commands.endeffector.EndEffectorScoreCoral;
import frc.robot.commands.swerveDrive.Align;
import frc.robot.libs.FieldAprilTags;
import frc.robot.libs.LoggedCommand;
import frc.robot.libs.NetworkTables;
import frc.robot.subsystems.odometry.Odometry;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionAndScoreCoral extends LoggedCommand {
  private Pose2d finalPose = null;
  private Command pathfindingCommand = null;
  private double scoreHeight = Constants.ElevatorHeights.minimum;
  /**
   * Creates a new ScoreCoral.
   * 
   * @param Elevator
   * @param Odometry
   * @param Position
   * @param reefSide
   */
  public PositionAndScoreCoral(String pos, boolean algae) {

    String[] posParts = pos.split("_");
    String side = posParts[0];
    String position = posParts[1];
    System.out.println("Side: " + side + ", Position: " + position);
    
    int reefSide = FieldAprilTags.getInstance().getClosestReefAprilTag(
        Odometry.getInstance().getPose(),
        DriverStation.getAlliance().get()).reefSide;
    scoreHeight = switch(position) {
        case "1" -> Constants.ElevatorHeights.reefCoralL1Height;
        case "2" -> algae ? Constants.ElevatorHeights.reefAlgaeL1Height : Constants.ElevatorHeights.reefCoralL2Height;
        case "3" -> algae ? Constants.ElevatorHeights.reefAlgaeL2Height : Constants.ElevatorHeights.reefCoralL3Height;
        default -> Constants.ElevatorHeights.reefCoralL4Height; 
    };
    int posint = switch(side) {
      case "L" -> -1;
      case "R" -> 1;
      default -> 0;
    };
    finalPose = Constants.ReefPoses.getPose(reefSide, algae ? -1 : posint);
    NetworkTables.pathplannerGoalPose.setDoubleArray(new double[] {
                finalPose.getX(),
                finalPose.getY(),
                finalPose.getRotation().getDegrees() });

  }
  @Override 
  public void initialize() {
    pathfindingCommand = AutoBuilder.pathfindToPose(
        finalPose,
        Constants.PathplannerConstants.pathplannerConstraints, 0.0)
        .andThen(
            new Align(finalPose)
            .alongWith(new SetHeight(scoreHeight)), new EndEffectorScoreCoral(0.5)); // TODO: Why doesn't the end effector run???
    pathfindingCommand.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  public boolean isFinished() {
    return pathfindingCommand != null ? pathfindingCommand.isFinished() : false;
  }
}