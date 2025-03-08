// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private static Elevator instance = null;

  public final SparkMax LMot;
  public final SparkMax RMot;

  private final Encoder LeftEncoder;
  private final Encoder RightEncoder;

  public final Constraints ElevConstraints = new Constraints(Constants.Constraints.elevatorMaxVelocity,
      Constants.Constraints.elevatorMaxAcceleration);
  // Two PIDs so I and D can be consistant per side.
  private final PIDController pidLeft;
  private final PIDController pidRight;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable ElevatorTable;
  private final NetworkTable ElevatorPIDs;
  private double target;

  private final SparkMaxConfig lConfig;
  private final SparkMaxConfig rConfig;

  private final double kP = 0.5;
  private final double kI = 0.0;
  private final double kD = 0.0;

  private double retainingPower = 0.01;

  private double speedL = 0;
  private double speedR = 0;

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  /** Creates a new Elevator. */
  private Elevator() {
    ElevatorTable = inst.getTable("Elevator");
    ElevatorPIDs = ElevatorTable.getSubTable("PID");

    LMot = new SparkMax(Constants.MotorIDs.ElevLNeo, MotorType.kBrushless);
    RMot = new SparkMax(Constants.MotorIDs.ElevRNeo, MotorType.kBrushless);

    lConfig = new SparkMaxConfig();
    lConfig.idleMode(IdleMode.kBrake);
    lConfig.inverted(true);

    rConfig = new SparkMaxConfig();
    rConfig.idleMode(IdleMode.kBrake);

    LMot.configure(lConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RMot.configure(rConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    LeftEncoder = new Encoder(Constants.SensorIDs.ElevEncoderLeftA, Constants.SensorIDs.ElevEncoderLeftB);// new
                                                                                                          // ElevatorOffsetDutyCycleEncoder(Constants.SensorIDs.ElevEncoderLeft,
                                                                                                          // false);
    RightEncoder = new Encoder(Constants.SensorIDs.ElevEncoderRightA, Constants.SensorIDs.ElevEncoderRightB, true); // new
                                                                                                                    // ElevatorOffsetDutyCycleEncoder(Constants.SensorIDs.ElevEncoderRight,
                                                                                                                    // true);

    ElevatorPIDs.getEntry("Left Encoder Distance").setDouble(LeftEncoder.getDistance());
    ElevatorPIDs.getEntry("Right Encoder Distance").setDouble(RightEncoder.getDistance());

    ElevatorPIDs.getEntry("P").setDouble(kP);
    ElevatorPIDs.getEntry("I").setDouble(kI);
    ElevatorPIDs.getEntry("D").setDouble(kD);

    ElevatorPIDs.getEntry("RP").setDouble(retainingPower);

    pidLeft = new PIDController(
        ElevatorPIDs.getEntry("P").getDouble(0),
        ElevatorPIDs.getEntry("I").getDouble(0),
        ElevatorPIDs.getEntry("D").getDouble(0));
    pidRight = new PIDController(
        ElevatorPIDs.getEntry("P").getDouble(0),
        ElevatorPIDs.getEntry("I").getDouble(0),
        ElevatorPIDs.getEntry("D").getDouble(0));

    ElevatorPIDs.getEntry("P").setPersistent();
    ElevatorPIDs.getEntry("I").setPersistent();
    ElevatorPIDs.getEntry("D").setPersistent();
    ElevatorPIDs.getEntry("RP").setPersistent();

    setZero();
  }

  private double calculateSpeed() {
    return (speedL + speedR) / 2;
  }

  private double getEncoderAverage() {
    return (LeftEncoder.getDistance() + RightEncoder.getDistance()) / 2;
  }

  @Override
  public void periodic() {
    ElevatorPIDs.getEntry("Left Encoder Distance").setDouble(LeftEncoder.getDistance() * Constants.Bot.elevatorEncoderDegreesToMeters);
    ElevatorPIDs.getEntry("Right Encoder Distance").setDouble(RightEncoder.getDistance() * Constants.Bot.elevatorEncoderDegreesToMeters);

    pidLeft.setP(ElevatorPIDs.getEntry("P").getDouble(0));
    pidLeft.setI(ElevatorPIDs.getEntry("I").getDouble(0));
    pidLeft.setD(ElevatorPIDs.getEntry("D").getDouble(0));

    pidRight.setP(ElevatorPIDs.getEntry("P").getDouble(0));
    pidRight.setI(ElevatorPIDs.getEntry("I").getDouble(0));
    pidRight.setD(ElevatorPIDs.getEntry("D").getDouble(0));

    retainingPower = ElevatorPIDs.getEntry("RP").getDouble(0);

    ElevatorTable.getEntry("Current Height").setDouble(getHeight());
    ElevatorTable.getEntry("Target Height").setDouble(target);

    pidLeft.setSetpoint(target);
    pidRight.setSetpoint(target);

    speedL = pidLeft.calculate(LeftEncoder.getDistance() * Constants.Bot.elevatorEncoderDegreesToMeters)
        + retainingPower;
    speedR = pidRight.calculate(RightEncoder.getDistance() * Constants.Bot.elevatorEncoderDegreesToMeters)
        + retainingPower;

    ElevatorPIDs.getEntry("Left Speed").setDouble(speedL);
    ElevatorPIDs.getEntry("Right Speed").setDouble(speedR);

    if (Controller.getInstance().getControlMode() == Controller.ControlMode.MANUAL)
      return;

    LMot.set(speedL);
    RMot.set(speedR);
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
    return getEncoderAverage() * Constants.Bot.elevatorEncoderDegreesToMeters;

  }

  public boolean isHome() {
    return Math.abs(LMot.getOutputCurrent()) > Constants.Limits.CurrentHomeThreshold;
  }

  public double getTarget() {
    return target;
  }

  public boolean isMoving() {
    return Math.abs(calculateSpeed()) > Constants.Limits.ElevMovement;
  }

  public Boolean[] isAtHeight(double height, double tolerance) {
    // 0: is within tolerance, 1: is staying in tolerance
    return new Boolean[] { Math.abs(getHeight() - height) < tolerance, Math.abs(target - height) < tolerance };
  }

  public void setZero() {
    LeftEncoder.reset();
    RightEncoder.reset();
  }
}