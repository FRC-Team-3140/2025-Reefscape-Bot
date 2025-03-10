// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReturnToStowed extends SequentialCommandGroup {
  /** Creates a new ReturnToStowed. */
  public ReturnToStowed() {
    super(new SetHeight(Constants.ElevatorHeights.safeStowed),
        new InstantCommand(() -> EndEffector.getInstance().setAlgaeIntakeAngle(Constants.AlgaeIntakeAngles.stowed)));
  }
}
