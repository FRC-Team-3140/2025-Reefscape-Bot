// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compoundCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.LEDs.setLEDColor;
import frc.robot.commands.LEDs.setRainbow;
import frc.robot.commands.elevator.SetHeight;
import frc.robot.commands.endeffector.EndEffectorIntakeAlgae;

public class GetAlgaeGroundManual extends SequentialCommandGroup {
  /** Creates a new SourceCoralIntake. */
  public GetAlgaeGroundManual(EndEffectorIntakeAlgae.Level level) {
    super(new SetHeight(Constants.ElevatorHeights.groundIntake), new setLEDColor(0, 255, 0),
        new EndEffectorIntakeAlgae(level), new setRainbow());
  }
}