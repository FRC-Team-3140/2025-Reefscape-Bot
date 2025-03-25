// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.libs.LoggedCommand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Align extends LoggedCommand {

  private final double transP = 8;
  private final double transI = 1;
  private final double transD = 0;

  private final double rotP = 6;
  private final double rotI = 1;
  private final double rotD = 0;

  private final PIDController xPID;
  private final PIDController yPID;
  private final PIDController thetaPID;

  private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

  private double transTolerance = 0.05; // meters
  private double rotTolerance = Math.toRadians(2); // radians

  private Pose2d currentPose = new Pose2d();
  private Pose2d targetPose;

  private Odometry odometry = Odometry.getInstance();

  public  Align(Pose2d targetPose) {
    this.targetPose = targetPose; 
    xPID = new PIDController(transP, transI, transD);
    yPID = new PIDController(transP, transI, transD);
    thetaPID = new PIDController(rotP, rotI, rotD);

    xPID.setSetpoint(targetPose.getX());
    yPID.setSetpoint(targetPose.getY());
    thetaPID.setSetpoint(targetPose.getRotation().getRadians());

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    super.initialize();
  }
  @Override
  public void execute() {
    currentPose = odometry.getPose();
    double driveX = xPID.calculate(currentPose.getX());
    double driveY = yPID.calculate(currentPose.getY());
    double driveTheta = thetaPID.calculate(currentPose.getRotation().getRadians());
    System.out.println("======");
    System.out.println(currentPose.getX() + ", " + currentPose.getY() + " : " + currentPose.getRotation().getRadians());
    System.out.println(targetPose.getX() + ", " + targetPose.getY() + " : " + targetPose.getRotation().getRadians());
    System.out.println(driveX + ", " + driveY + " : " + driveTheta);
    swerveDrive.drive(driveX, driveY, driveTheta, true);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    swerveDrive.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return currentPose.getTranslation().getDistance(targetPose.getTranslation()) < transTolerance && 
      Math.abs(currentPose.getRotation().getRadians() - currentPose.getRotation().getRadians()) < rotTolerance;
  }
}

