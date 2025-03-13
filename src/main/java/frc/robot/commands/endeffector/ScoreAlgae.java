// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.SetHeight;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreAlgae extends SequentialCommandGroup {
  private final EndEffector endEffector;

  /** Creates a new ScoreAlgae. */
  public ScoreAlgae() {
    this.endEffector = EndEffector.getInstance();

    addCommands(new SetHeight(Constants.ElevatorHeights.groudAlgaeHeight), new ScoreTheCORAL());
  }

  private class ScoreTheCORAL extends Command {
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      endEffector.setAlgaeIntakeAngle(Constants.AlgaeIntakeAngles.processorScoreBottom);
      Timer.delay(0.5);
      endEffector.setAlgaeIntakeSpeed(0.75);
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
      Timer.delay(1.25);
      endEffector.setAlgaeIntakeSpeed(0);
      return true;
    }
  }
}
