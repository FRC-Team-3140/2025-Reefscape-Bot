// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeElevator extends Command {
  /** Creates a new HomeElevator. */
  Elevator elev;
  double up = 0;
  double lastH = 0;

  public HomeElevator() {
    elev = Elevator.getInstance();
    addRequirements(elev);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    up = Constants.ElevatorHeights.homeUpDist;
    lastH = elev.getHeight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (up != 0) {
      double delta = elev.getHeight() - lastH;
      lastH += delta;
      up = Math.max(up - delta, 0);
      elev.setHeight(elev.getHeight() + 100, false);
    } else {
      elev.setHeight(elev.getHeight() - 100, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      elev.zero();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return up == 0 && elev.isHome();
  }
}
