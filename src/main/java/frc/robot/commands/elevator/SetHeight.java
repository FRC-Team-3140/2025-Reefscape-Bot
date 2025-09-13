// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import frc.robot.libs.LoggedCommand;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetHeight extends LoggedCommand {
  private final Elevator elev;
  private final double height;
  private double startTStamp;

  /** Creates a new SetHeight. */
  public SetHeight(double height) {
    this.height = height;
    this.elev = Elevator.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    if (!this.getRequirements().contains(elev))
      addRequirements(elev);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    this.startTStamp = Timer.getFPGATimestamp();
    elev.setHeight(height);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (Timer.getFPGATimestamp() - startTStamp > 2)
          || ((Timer.getFPGATimestamp() - startTStamp > 0.2) && !elev.isMoving());
  }
}
