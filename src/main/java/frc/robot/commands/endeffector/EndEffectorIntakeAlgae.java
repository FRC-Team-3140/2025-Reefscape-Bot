// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endeffector;

import frc.robot.Constants;
import frc.robot.libs.LoggedCommand;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorIntakeAlgae extends LoggedCommand {
  private EndEffector endEffector = null;
  private Elevator elevator = null;
  private final Level level;

  public enum Level {
    Ground,
    AlgaeL1,
    AlgaeL2
  }

  /** Creates a new IntakeCoral. */
  public EndEffectorIntakeAlgae(Level level) {
    this.endEffector = EndEffector.getInstance();
    this.elevator = Elevator.getInstance();
    this.level = level;

    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (level == Level.AlgaeL1) {
      elevator.setHeight(Constants.ElevatorHeights.reefAlgaeL1Height);
    } else if (level == Level.AlgaeL2) {
      elevator.setHeight(Constants.ElevatorHeights.reefAlgaeL2Height);
    } else if (level == Level.Ground) {
      elevator.setHeight(Constants.ElevatorHeights.groudAlgaeHeight);
    } else {
      System.err.println("Invalid position for End Effector Intake Algae.");
      end(true);
    }

    if (level == Level.Ground) {
      endEffector.setAlgaeIntakeAngle(Constants.AlgaeIntakeAngles.groundIntake);
    } else {
      endEffector.setAlgaeIntakeAngle(Constants.AlgaeIntakeAngles.reefIntake);
    }

    endEffector.setAlgaeIntakeSpeed(-0.9);

    Timer.delay(0.1);

    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Timer.delay(0.25);
    endEffector.setAlgaeIntakeSpeed(-0.15);
    System.out.println("EndEffectorIntakeAlgae ended");
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endEffector.getAlgaeIntakeCurrent() > Constants.Limits.EEAlgaeIntakeCurrentThreshold;
  }
}
