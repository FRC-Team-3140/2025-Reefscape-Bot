// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SignalTower;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setRainbow extends Command {
  /** Creates a new setRainbow. */
  public setRainbow() {
    SignalTower.getInstance().setRainbow();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
