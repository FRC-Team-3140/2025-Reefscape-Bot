// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.elevator.SetHeight;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreAlgae extends ParallelCommandGroup {
  private final EndEffector endEffector;

  /** Creates a new ScoreAlgae. */
  public ScoreAlgae() {
    this.endEffector = EndEffector.getInstance();

    addCommands(new SetHeight(Constants.ElevatorHeights.groudAlgaeHeight), new ScoreTheCORAL());
  }

  private class ScoreTheCORAL extends SequentialCommandGroup {
    // Called when the command is initially scheduled.
    public ScoreTheCORAL() {
      addCommands(
          new InstantCommand(() -> endEffector.setAlgaeIntakeAngle(Constants.AlgaeIntakeAngles.processorScoreBottom)),
          new InstantCommand(() -> endEffector.setAlgaeIntakeSpeed(0.75)),
          new WaitCommand(2),
          new InstantCommand(() -> {
            endEffector.setAlgaeIntakeAngle(Constants.AlgaeIntakeAngles.stowed);
            endEffector.setAlgaeIntakeSpeed(0);
          }));
    }
  }
}
