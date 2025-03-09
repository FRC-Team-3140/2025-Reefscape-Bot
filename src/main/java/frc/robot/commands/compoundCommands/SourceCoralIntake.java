// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compoundCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.LEDs.setLEDColor;
import frc.robot.commands.LEDs.setRainbow;
import frc.robot.commands.elevator.SetHeight;
import frc.robot.commands.endeffector.EndEffectorIntakeCoral;

// TODO: test if squential works (instead of parallel)
public class SourceCoralIntake extends SequentialCommandGroup {
  /** Creates a new SourceCoralIntake. */
  public SourceCoralIntake() {
    super(new setLEDColor(255, 0, 0), new SetHeight(Constants.ElevatorHeights.sourceIntake), new setLEDColor(0, 255, 0), new EndEffectorIntakeCoral(), new setRainbow()) ;
  }
}