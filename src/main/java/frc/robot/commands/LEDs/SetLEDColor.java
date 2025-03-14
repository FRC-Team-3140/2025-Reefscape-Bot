// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import frc.robot.libs.LoggedCommand;
import frc.robot.subsystems.SignalTower;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetLEDColor extends LoggedCommand {
  private final int R;
  private final int G;
  private final int B;

  /** Creates a new setLEDColor. */
  public SetLEDColor(int R, int G, int B) {
    this.R = R;
    this.G = G;
    this.B = B;
  }

  @Override
  public void initialize() {
    super.initialize();
    SignalTower.getInstance().setSolid(R, G, B);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if (interrupted) {
      SignalTower.getInstance().setSolid(0,0,0);
      Integer i = null;
      i.byteValue();
    }
    
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
