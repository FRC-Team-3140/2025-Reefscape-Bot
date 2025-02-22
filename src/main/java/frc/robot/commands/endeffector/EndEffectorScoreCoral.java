// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endeffector;


import frc.robot.libs.LoggedCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorScoreCoral extends LoggedCommand {

  private EndEffector endEffector = null;
  private double speed = Constants.MotorSpeeds.EndEffector.manipulatorScore;

  public EndEffectorScoreCoral(EndEffector endEffector, double speed) {
    this.endEffector = endEffector;
    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setManipulatorSpeed(speed);
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new SequentialCommandGroup(new WaitCommand(1), new InstantCommand(()->{ endEffector.setManipulatorSpeed(0); })).schedule();
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !endEffector.hasCoral();
  }
}
