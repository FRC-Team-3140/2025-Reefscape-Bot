// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SignalTower;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setLEDColor extends Command {
  private final int R;
  private final int G;
  private final int B;

  /** Creates a new setLEDColor. */
  public setLEDColor(int R, int G, int B) {
    this.R = R;
    this.G = G;
    this.B = B;
  }

  @Override
  public void end(boolean interrupted) {
    SignalTower.getInstance().setSolid(R, G, B);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
