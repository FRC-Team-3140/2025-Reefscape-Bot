// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.libs.LoggedCommand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToClosestSourceDiffered extends LoggedCommand {
  private GoToClosestSource command = null;

  public GoToClosestSourceDiffered() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    command = new GoToClosestSource();
    command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (command != null) {
      command.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (command != null && interrupted) {
      command.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (command != null) && command.isFinished();
  }
}
