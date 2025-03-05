// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CycleHorizontal;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.odometry.Odometry;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CycleL4 extends Command {
  private Elevator elev = null;
  private Odometry odometry = null;

  /** Creates a new CycleL4. */
  public CycleL4(Elevator elev, Odometry odometry) {
    this.elev = elev;
    this.odometry = odometry;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elev, odometry);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new CycleHorizontal(4, elev, odometry).schedule();
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
    return true;
  }
}
