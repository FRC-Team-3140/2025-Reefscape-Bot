// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class IntakeAlgaeReef extends Command {
  EndEffector endEffector = null;
  double height = Constants.ElevatorHeights.minimum;

  public IntakeAlgaeReef(EndEffector endEffector, double intakeHeight) {
    this.endEffector = endEffector;
    height = intakeHeight;

    addRequirements(endEffector);
  }

  @Override
  public void initialize() {
    Elevator.getInstance().setHeight(height);
    endEffector.setAlgaeIntakeAngle(Constants.AlgaeIntakeAngles.reefIntake);
  }

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
    return endEffector.getAlgaeIntakeCurrent() > Constants.Limits.EEIntakeCurrentThreshold;
  }
}
