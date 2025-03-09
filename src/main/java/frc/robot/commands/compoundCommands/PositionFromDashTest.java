// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compoundCommands;
import frc.robot.Constants.ElevatorHeights;
import frc.robot.commands.elevator.SetHeight;
import frc.robot.libs.LoggedCommand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionFromDashTest extends LoggedCommand {
  private Double level = null;

  /**
   * Creates a new ScoreCoral.
   * 
   * @param Elevator
   * @param Odometry
   * @param Position
   * @param reefSide
   */
  public PositionFromDashTest(String pos) {
    String[] posParts = pos.split("");
    String position = posParts[1];
    System.out.println("Position: " + position);

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
        break;
    }
  }
  @Override
  public void end(boolean interrupted) {
    new SetHeight(level).schedule();
    super.end(interrupted);
  } 

  public boolean isFinished() {
    return true;
  }
}