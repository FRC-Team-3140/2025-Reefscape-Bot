// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compoundCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToClosestSource;
import frc.robot.commands.endeffector.EndEffectorIntakeCoral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToSourceAndIntake extends SequentialCommandGroup {
  /** Creates a new GoToSourceAndIntake. */
  public GoToSourceAndIntake() {
    super(new SourceCoralIntake(), new GoToClosestSource(), new EndEffectorIntakeCoral());
  }
}