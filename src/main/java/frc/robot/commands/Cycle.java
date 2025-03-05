// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.odometry.Odometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cycle extends SequentialCommandGroup {
  /** Creates a new Cycle. */
  public Cycle(int level, Elevator elevator, Odometry odometry) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    for (int i = 0; i < 5; i++) {
      super.addCommands(
          new ScoreCoral(elevator, odometry, ScoreCoral.Position.valueOf("R_" + level), i),
          new GoToClosestSource(odometry),
          new ScoreCoral(elevator, odometry, ScoreCoral.Position.valueOf("L_" + level), i));
    }
  }
}
