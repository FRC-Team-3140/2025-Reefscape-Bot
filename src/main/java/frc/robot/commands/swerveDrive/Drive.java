// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  private final SwerveDrive swerve = SwerveDrive.getInstance();

  private final double startTime = System.currentTimeMillis();

  private final double duration;
  private final boolean fieldRel;
  private final double x;
  private final double y;
  private final double rot;

  /** Creates a new Drive. */
  public Drive(double duration, boolean fieldRel, double x, double y, double rot) {
    this.duration = duration;
    this.fieldRel = fieldRel;
    this.x = x;
    this.y = y;
    this.rot = rot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(x, y, rot, fieldRel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime > duration * 1000;
  }
}
