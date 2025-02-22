// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.libs.LoggedCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class IntakeAlgaeReef extends LoggedCommand {
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
    endEffector.setAlgaeIntakeSpeed(Constants.MotorSpeeds.EndEffector.algaeIntakeSpeed);
    super.initialize();
  }

  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setAlgaeIntakeSpeed(0);
    endEffector.setAlgaeIntakeAngle(Constants.AlgaeIntakeAngles.stowedAlgaeTop);
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endEffector.getAlgaeIntakeCurrent() > Constants.Limits.EEIntakeCurrentThreshold;
  }
}
