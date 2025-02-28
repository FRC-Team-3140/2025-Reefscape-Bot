// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private static Elevator instance = null;

  private final SparkMax LMot;
  private final SparkMax RMot;
  private final SparkAbsoluteEncoder Enc;
  private final ProfiledPIDController pid;
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable ElevatorTable;
  private final NetworkTable ElevatorPIDs;
  private double target;
  private double enc_offset;

  private final SparkMaxConfig lConfig;
  private final SparkMaxConfig rConfig;

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  /** Creates a new Elevator. */
  private Elevator() {
    enc_offset = 0;

    ElevatorTable = inst.getTable("Elevator");
    ElevatorPIDs = ElevatorTable.getSubTable("PID");

    LMot = new SparkMax(Constants.MotorIDs.ElevLNeo, MotorType.kBrushless);
    RMot = new SparkMax(Constants.MotorIDs.ElevRNeo, MotorType.kBrushless);

    lConfig = new SparkMaxConfig();
    lConfig.idleMode(IdleMode.kBrake);

    rConfig = new SparkMaxConfig();
    rConfig.idleMode(IdleMode.kBrake);
    rConfig.follow(LMot, true);

    LMot.configure(lConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RMot.configure(rConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Enc = LMot.getAbsoluteEncoder();

    pid = new ProfiledPIDController(
        ElevatorPIDs.getEntry("P").getDouble(0),
        ElevatorPIDs.getEntry("I").getDouble(0),
        ElevatorPIDs.getEntry("D").getDouble(0),
        Constants.Constraints.ElevConstraints);

    ElevatorPIDs.getEntry("P").setPersistent();
    ElevatorPIDs.getEntry("I").setPersistent();
    ElevatorPIDs.getEntry("D").setPersistent();
  }

  public void zero() {
    enc_offset = Enc.getPosition();
  }

  private double calculateSpeed() {
    return pid.calculate(getHeight(), new TrapezoidProfile.State(target, 0));
  }

  @Override
  public void periodic() {
    pid.setP(ElevatorPIDs.getEntry("P").getDouble(0));
    pid.setI(ElevatorPIDs.getEntry("I").getDouble(0));
    pid.setD(ElevatorPIDs.getEntry("D").getDouble(0));

    ElevatorTable.getEntry("Current Height").setDouble(getHeight());
    ElevatorTable.getEntry("Target Height").setDouble(target);

    double speed = calculateSpeed();
    LMot.set(speed);
  }

  public void setHeight(double height) {
    target = Math.max(Math.min(height, Constants.ElevatorHeights.maxiumum), Constants.ElevatorHeights.minimum);
  }

  public void setHeight(double height, boolean override) {
    if (override) {
      target = height;
    } else {
      setHeight(height);
    }
  }

  public double getHeight() {
    return (Enc.getPosition() - enc_offset)*Constants.ElevatorHeights.conversionFactor;
  }

  public boolean isHome() {
    return Math.abs(LMot.getOutputCurrent()) > Constants.Limits.CurrentHomeThreshold;
  }

  public boolean isMoving() {
    return Math.abs(calculateSpeed()) > Constants.Limits.ElevMovement;
  }

  public Boolean[] isAtHeight(double height, double tolerance) {
    // 0: is within tolerance, 1: is staying in tolerance
    return new Boolean[] { Math.abs(getHeight() - height) < tolerance, Math.abs(target - height) < tolerance };
  }
}