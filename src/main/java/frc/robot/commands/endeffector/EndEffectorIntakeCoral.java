// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endeffector;

import frc.robot.libs.LoggedCommand;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorIntakeCoral extends LoggedCommand {
  private EndEffector endEffector = null;

  /** Creates a new IntakeCoral. */
  public EndEffectorIntakeCoral() {
    this.endEffector = EndEffector.getInstance();

    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setManipulatorSpeed(0.25);
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Timer.delay(0.25);
    endEffector.setManipulatorSpeed(0);
    System.out.println("EndEffectorIntakeCoral ended");
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endEffector.hasCoral();
  }
}
