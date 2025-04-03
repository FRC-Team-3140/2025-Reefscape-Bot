// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.libs.FieldAprilTags;
import frc.robot.libs.LoggedCommand;
import frc.robot.libs.NetworkTables;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Align extends SequentialCommandGroup {
  private final double transP = 8;
  private final double transI = 0;
  private final double transD = 0;

  private final double rotP = 1;
  private final double rotI = 0;
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

  private double startTime;

  private double maxDuration = 7;

  public Align(Pose2d targetPose) {
    this.targetPose = targetPose;
    xPID = new PIDController(transP, transI, transD);
    yPID = new PIDController(transP, transI, transD);
    thetaPID = new PIDController(rotP, rotI, rotD);

    xPID.setSetpoint(targetPose.getX());
    yPID.setSetpoint(targetPose.getY());
    thetaPID.setSetpoint(targetPose.getRotation().getRadians() + (DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Math.PI : 0));

    thetaPID.enableContinuousInput(-Math.PI, Math.PI);

    addCommands(new AlignCommand(targetPose),
        new Drive(1000, true, 0.7 * Math.cos(thetaPID.getSetpoint()), 0.7 * Math.sin(thetaPID.getSetpoint()), 0));
  }

  public Align() {
    this(FieldAprilTags.getInstance().getTagPose(
        FieldAprilTags.getInstance().getClosestReefAprilTag(
            Odometry.getInstance().getPose(),
            DriverStation.getAlliance().get()).aprilTag.ID));
  }

  private class AlignCommand extends LoggedCommand {
    public AlignCommand(Pose2d targetPose) {
      addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
      startTime = Timer.getFPGATimestamp();
      NetworkTables.pathplannerGoalPose.setDoubleArray(new double[] {
        targetPose.getX(),
        targetPose.getY(),
        targetPose.getRotation().getDegrees() });
      super.initialize();
    }

    @Override
    public void execute() {
      currentPose = odometry.getPose();
      double driveX = xPID.calculate(currentPose.getX());
      double driveY = yPID.calculate(currentPose.getY());
      double driveTheta = thetaPID.calculate(odometry.getRotation().getRadians());
      System.out.println("Driving to " + thetaPID.getSetpoint() + " from " + odometry.getRotation().getRadians()
          + " by driving: " + driveTheta);
      swerveDrive.drive(driveX, driveY, driveTheta, true);
    }

    @Override
    public void end(boolean interrupted) {
      super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
      if (startTime + 1 > Timer.getFPGATimestamp())
        return false;
      if (startTime + maxDuration < Timer.getFPGATimestamp())
        return true;
      if ((currentPose.getTranslation().getDistance(targetPose.getTranslation()) < transTolerance &&
          Math.abs(currentPose.getRotation().getRadians() - currentPose.getRotation().getRadians()) < rotTolerance) ||
          !odometry.isMoving())
        return true;
      return false;
    }
  }
}