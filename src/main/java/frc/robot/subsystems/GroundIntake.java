// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.elevator.SetHeight;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

public class GroundIntake extends SubsystemBase {
  private static GroundIntake instance;

  private static Elevator elevator = Elevator.getInstance();

  private final SparkMax leftMN;
  private final SparkMax rightMN;
  private final SparkMax liftN;
  private final DutyCycleEncoder liftPos;

  private ProfiledPIDController liftPID;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTableEntry pEntry;
  private NetworkTableEntry iEntry;
  private NetworkTableEntry dEntry;

  private double setPoint = 0;

  private boolean intakeActive = false;
  private boolean coralHeld = false;

  private SparkMaxConfig leftMNCoast = new SparkMaxConfig();
  private SparkMaxConfig leftMNBrake = new SparkMaxConfig();

  private SparkMaxConfig rightMNCoast = new SparkMaxConfig();
  private SparkMaxConfig rightMNBrake = new SparkMaxConfig();

  private SparkMaxConfig liftConfig = new SparkMaxConfig();

  public static class SetPoints {
    public static final double stow = 0;
    public static final double intake = 0;
    public static final double handoff = 0;
    public static final double maintainance = 0;
  }

  public static GroundIntake getInstance() {
    if (instance == null) {
      instance = new GroundIntake();
    }

    return new GroundIntake();
  }

  /** Creates a new GroundIntake. */
  private GroundIntake() {
    leftMN = new SparkMax(Constants.MotorIDs.GILeft, MotorType.kBrushless);
    rightMN = new SparkMax(Constants.MotorIDs.GIRight, MotorType.kBrushless);
    liftN = new SparkMax(Constants.MotorIDs.GILift, MotorType.kBrushless);

    leftMNCoast.idleMode(IdleMode.kCoast).inverted(false);
    leftMNBrake.idleMode(IdleMode.kBrake).inverted(false);

    rightMNCoast.idleMode(IdleMode.kCoast).inverted(true);
    rightMNBrake.idleMode(IdleMode.kBrake).inverted(true);

    liftConfig.idleMode(IdleMode.kBrake).inverted(false);

    leftMN.configure(leftMNCoast, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightMN.configure(rightMNCoast, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    liftN.configure(liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    liftPos = new DutyCycleEncoder(Constants.SensorIDs.GIEncoder);

    pEntry = inst.getTable("Ground Intake").getSubTable("PID Values").getEntry("P: ");
    iEntry = inst.getTable("Ground Intake").getSubTable("PID Values").getEntry("I: ");
    dEntry = inst.getTable("Ground Intake").getSubTable("PID Values").getEntry("D: ");

    pEntry.setPersistent();
    iEntry.setPersistent();
    dEntry.setPersistent();

    double p = inst.getTable("Ground Intake").getSubTable("PID Values").getEntry("P: ").getDouble(0);
    double i = inst.getTable("Ground Intake").getSubTable("PID Values").getEntry("I: ").getDouble(0);
    double d = inst.getTable("Ground Intake").getSubTable("PID Values").getEntry("D: ").getDouble(0);

    liftPID = new ProfiledPIDController(p, i, d, Constants.Constraints.GIConstraints);
  }

  @Override
  public void periodic() {
    liftPID.setP(inst.getTable("Ground Intake").getSubTable("PID Values").getEntry("P: ").getDouble(0));
    liftPID.setP(inst.getTable("Ground Intake").getSubTable("PID Values").getEntry("I: ").getDouble(0));
    liftPID.setP(inst.getTable("Ground Intake").getSubTable("PID Values").getEntry("D: ").getDouble(0));

    // This method will be called once per scheduler run
    liftN.set(liftPID.calculate(
        liftPos.get(),
        new TrapezoidProfile.State(
            (Math.min(Math.max(Constants.Limits.GIMinAngle, setPoint),
                Constants.Limits.GIMaxAngle)),
            0)));

    if (intakeActive) {
      if (leftMN.getOutputCurrent() >= Constants.Limits.GICoralDetectionCurrentThreshold
          || rightMN.getOutputCurrent() >= Constants.Limits.GICoralDetectionCurrentThreshold) {
        Boolean[] elevatorStats = elevator.isAtHeight(Constants.ElevatorHeights.minimum,
            Constants.Limits.ElevPosThreshold);
        new SequentialCommandGroup(
            (elevatorStats[0] && elevatorStats[1] ? null : new SetHeight(Constants.ElevatorHeights.minimum)),
            new InstantCommand(() -> {
              stopIntake();
              setAngle(SetPoints.stow);
              coralHeld = true;
            })).schedule();
      }
      // TODO: Finish implementing this logic as the command above
      if (coralHeld && elevator.isAtHeight(Constants.ElevatorHeights.minimum, Constants.Limits.ElevPosThreshold)[0]) {
        setAngle(SetPoints.handoff);
        outtake();
        Timer.delay(1);
        stopIntake();
        coralHeld = false;
      }
    }
  }

  public void intake() {
    intakeActive = true;

    setAngle(SetPoints.intake);

    leftMN.setVoltage(Constants.Voltages.GIVoltage);
    rightMN.setVoltage(Constants.Voltages.GIVoltage);
  }

  public void outtake() {
    intakeActive = false;

    leftMN.setVoltage(-Constants.Voltages.GIVoltage);
    rightMN.setVoltage(-Constants.Voltages.GIVoltage);
  }

  public void stopIntake() {
    leftMN.setVoltage(0);
    rightMN.setVoltage(0);
  }

  // Set Angle
  public void setAngle(double degree) {
    setPoint = degree;
  }
}