// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LEDs.SetRainbow;
import frc.robot.libs.LoggedCommand;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorIntakeCoral extends LoggedCommand {
  private EndEffector endEffector = null;

  /** Creates a new IntakeCoral. */
  public EndEffectorIntakeCoral() {
    this.endEffector = EndEffector.getInstance();

    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setManipulatorSpeed(0.25);
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    if (interrupted)
      new SetRainbow().schedule();

    new SequentialCommandGroup(new WaitCommand(0.25),
        new InstantCommand(
            () -> {
              endEffector.setManipulatorSpeed(0);
            }))
        .schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotBase.isSimulation())
      return true;
    return endEffector.hasCoral();
  }
}
