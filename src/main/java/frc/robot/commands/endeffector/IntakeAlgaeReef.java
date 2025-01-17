// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class IntakeAlgaeReef extends Command {
  EndEffector endEffector = EndEffector.getInstance();

  public IntakeAlgaeReef() {
    addRequirements(endEffector);
  }

  @Override
  public void initialize() {
    endEffector.setAlgaeIntakeAngle(Constants.AlgaeIntakeAngles.reefIntake);
  }

  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
