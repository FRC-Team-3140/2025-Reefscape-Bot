// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.libs.LoggedCommand;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeAlgae extends LoggedCommand {
  private final EndEffector endEffector;
  private double startTimestamp;

  /** Creates a new OuttakeAlgae. */
  public OuttakeAlgae() {
    this.endEffector = EndEffector.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    if (!this.getRequirements().contains(endEffector))
      addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    endEffector.setAlgaeIntakeSpeed(1);
    startTimestamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    endEffector.setAlgaeIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTimestamp > 0.25;
  }
}
