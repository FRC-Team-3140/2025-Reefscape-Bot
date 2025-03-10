// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveDrive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setSwerveStates extends Command {
  private SwerveDrive swerve = null;

  private SwerveModuleState[] state = null;

  /**
   * Creates a new setSwerveStates command.
   * 
   * @param swerve The SwerveDrive subsystem used by this command.
   * 
   *               If no state is passed, it will use the default state in
   *               Constants.java
   */
  public setSwerveStates(SwerveDrive swerve) {
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  /**
   * Creates a new setSwerveStates command.
   * 
   * @param swerve The SwerveDrive subsystem used by this command.
   * @param state  An array of SwerveModuleStates FL, FR, BL, BR
   * 
   *               Will use the provided state
   */
  public setSwerveStates(SwerveDrive swerve, SwerveModuleState[] state) {
    this.swerve = swerve;
    if (state.length == swerve.modules.length) {
      this.state = state;
    } else {
      // In this scenario, the code will continue with the default states like the
      // constructor
      // above.
      System.err.println("resetSwerveStates was called with an invalid state array.");
    }

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
    swerve.setSwerveModuleStates(state == null ? Constants.Bot.defaultSwerveStates : state, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
