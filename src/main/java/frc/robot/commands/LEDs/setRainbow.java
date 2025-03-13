// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import frc.robot.libs.LoggedCommand;
import frc.robot.subsystems.SignalTower;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetRainbow extends LoggedCommand {
  /** Creates a new setRainbow. */
  public SetRainbow() {

  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    SignalTower.getInstance().setRainbow();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
