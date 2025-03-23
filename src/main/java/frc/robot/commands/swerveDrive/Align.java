// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveDrive;

import frc.robot.libs.LoggedCommand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Align extends LoggedCommand {
  // TODO: Finish implementation
  private final double x;
  private final double y;
  private final double theta;

  private final double transP = 5;
  private final double transI = 5;
  private final double transD = 5;

  /** Creates a new setLEDColor. */
  public Align(double x, double y, double theta) {
    this.x = x;
    this.y = y;
    this.theta = theta;
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

