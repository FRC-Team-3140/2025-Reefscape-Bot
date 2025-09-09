// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.libs.LoggedCommand;
import frc.robot.subsystems.SwerveDrive;

public class PathfindFromCurrentPose extends LoggedCommand {
  private final Pose2d targetPose;
  private final PathConstraints pathplannerconstraints;
  private final double endVelocity;
  private Command pathfindCommand;

  /** Creates a new PathfindFromCurrentPose. */
  public PathfindFromCurrentPose(Pose2d targetPose, PathConstraints pathplannerconstraints, double endVelocity) {
    this.targetPose = targetPose;
    this.pathplannerconstraints = pathplannerconstraints;
    this.endVelocity = endVelocity;

    if (!this.getRequirements().contains(SwerveDrive.getInstance()))
      addRequirements(SwerveDrive.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Generate the pathfinding command
    pathfindCommand = AutoBuilder.pathfindToPose(targetPose, pathplannerconstraints, endVelocity);

    // Schedule the generated command
    pathfindCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pathfindCommand != null) {
      pathfindCommand.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (pathfindCommand != null && interrupted) {
      pathfindCommand.cancel();
    }
    SwerveDrive.getInstance().drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathfindCommand.isFinished();
  }
}
