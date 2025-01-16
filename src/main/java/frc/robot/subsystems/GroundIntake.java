// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

public class GroundIntake extends SubsystemBase {
  private static GroundIntake instance;

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

  private SparkMaxConfig leftMNCoast = new SparkMaxConfig();
  private SparkMaxConfig leftMNBrake = new SparkMaxConfig();

  private SparkMaxConfig rightMNCoast = new SparkMaxConfig();
  private SparkMaxConfig rightMNBrake = new SparkMaxConfig();

  private SparkMaxConfig liftConfig = new SparkMaxConfig();

  public class setPoint {
    public static final double stow = 0;
    public static final double intake = 0;
    public static final double maintainance = 0;
  }

  public GroundIntake getInstance() {
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
        stopIntake();
      }
    }
  }

  public void intake() {
    intakeActive = true;

    leftMN.configure(leftMNCoast, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMN.configure(leftMNCoast, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    leftMN.setVoltage(Constants.Voltages.GIVoltage);
    rightMN.setVoltage(Constants.Voltages.GIVoltage);
  }

  public void outtake() {
    intakeActive = false;

    leftMN.configure(leftMNCoast, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMN.configure(leftMNCoast, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    leftMN.setVoltage(-Constants.Voltages.GIVoltage);
    rightMN.setVoltage(-Constants.Voltages.GIVoltage);
  }

  public void stopIntake() {
    leftMN.configure(leftMNBrake, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMN.configure(leftMNBrake, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    leftMN.setVoltage(0);
    rightMN.setVoltage(0);
  }

  // Set Angle
  public void setAngle(double degree) {
    setPoint = degree;
  }
}