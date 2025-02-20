// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WaitUntilAngle extends Command {
  GroundIntake groundIntake;
  double angle = 0;
  double tolerance = 180;
  public WaitUntilAngle(GroundIntake groundIntake, double angle, double tolerance) {
    this.groundIntake = groundIntake;
    addRequirements(groundIntake);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(groundIntake.getAngle() - angle) < tolerance;
  }
}
