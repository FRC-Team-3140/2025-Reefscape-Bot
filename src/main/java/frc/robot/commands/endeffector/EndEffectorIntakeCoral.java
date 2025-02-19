// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorIntakeCoral extends Command {
  private EndEffector endEffector = null;

  /** Creates a new IntakeCoral. */
  public EndEffectorIntakeCoral(EndEffector endEffector) {
    this.endEffector = endEffector;

    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setBeltSpeed(Constants.MotorSpeeds.EndEffector.beltIntake);
    endEffector.setManipulatorSpeed(Constants.MotorSpeeds.EndEffector.manipulatorIntake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setBeltSpeed(0);
    endEffector.setManipulatorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endEffector.hasCoral();
  }
}
