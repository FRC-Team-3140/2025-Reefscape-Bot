// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.libs.LoggedCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundCoralIntake extends LoggedCommand {
  /** Creates a new GroundCoralIntake. */
  public GroundCoralIntake(GroundIntake groundIntake, Elevator elevator) {
    groundIntake.intake();
    elevator.setHeight(Constants.ElevatorHeights.groundIntake);
    addRequirements(groundIntake, elevator);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
