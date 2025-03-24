// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.libs.LoggedCommand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Align extends LoggedCommand {
  // TODO: Finish implementation
  private final double x;
  private final double y;
  private final double theta;

  private final double transP = 5;
  private final double transI = 0;
  private final double transD = 0;

  private final double rotP = 5;
  private final double rotI = 0;
  private final double rotD = 0;

  private final PIDController xPID;
  private final PIDController yPID;
  private final PIDController thetaPID;

  private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

  private double transTolerance = 0.04; // meters
  private double rotTolerance = Math.toRadians(2); // radians

  private Pose2d currentPose = new Pose2d();
  private Pose2d targetPose;

  public Align(double x, double y, double theta) {
    this.x = x;
    this.y = y;
    this.theta = theta;
    
    xPID = new PIDController(transP, transI, transD);
    yPID = new PIDController(transP, transI, transD);
    thetaPID = new PIDController(rotP, rotI, rotD);

    xPID.setSetpoint(x);
    yPID.setSetpoint(y);
    thetaPID.setSetpoint(theta);

    targetPose = new Pose2d(x, y, new Rotation2d(theta));
  
  }

  @Override
  public void initialize() {
    super.initialize();
  }
  @Override
  public void execute() {
    currentPose = Odometry.getInstance().getPose();
    swerveDrive.drive(xPID.calculate(currentPose.getX()), yPID.calculate(currentPose.getY()), thetaPID.calculate(theta), true);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return currentPose.getTranslation().getDistance(targetPose.getTranslation()) < transTolerance && Math.abs(theta - currentPose.getRotation().getRadians()) < rotTolerance;
  }
}

