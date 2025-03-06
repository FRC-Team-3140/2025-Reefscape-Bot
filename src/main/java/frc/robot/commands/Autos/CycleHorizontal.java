// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToClosestSource;
import frc.robot.commands.ScoreCoral;
import frc.robot.libs.FeildAprilTags;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.odometry.Odometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CycleHorizontal extends SequentialCommandGroup {
  /**
   * Creates a new Cycle.
   * 
   * @param level
   * @param elevator
   * @param odometry
   */
  public CycleHorizontal(int level, Elevator elevator, Odometry odometry) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    int startingSide = FeildAprilTags.getInstance().getClosestReefAprilTag(odometry.getPose(),
        DriverStation.getAlliance().get()).reefSide;

    // Loop through reef sides to build a complete auto (Won't fully complete bc of
    // time limit)
    for (int i = 0; i <= 5; i++) {
      super.addCommands(
          new ScoreCoral(elevator, odometry, ScoreCoral.Position.valueOf("R_" + level), ((i + startingSide) % 6)),
          new GoToClosestSource(odometry),
          new ScoreCoral(elevator, odometry, ScoreCoral.Position.valueOf("L_" + level), ((i + startingSide) % 6)),
          new GoToClosestSource(odometry));
    }
  }

  /**
   * Override starting side
   * 
   * @param level
   * @param startingSide
   * @param elevator
   * @param odometry
   */
  public CycleHorizontal(int level, int startingSide, Elevator elevator, Odometry odometry) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Loop through reef sides to build a complete auto (Won't fully complete bc of
    // time limit)
    for (int i = 0; i <= 5; i++) {
      super.addCommands(
          new ScoreCoral(elevator, odometry, ScoreCoral.Position.valueOf("R_" + level), ((i + startingSide) % 6)),
          new GoToClosestSource(odometry),
          new ScoreCoral(elevator, odometry, ScoreCoral.Position.valueOf("L_" + level), ((i + startingSide) % 6)),
          new GoToClosestSource(odometry));
    }
  }
}
