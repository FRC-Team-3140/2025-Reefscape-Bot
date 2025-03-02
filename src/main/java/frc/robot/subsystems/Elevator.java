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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.AbsoluteEncoder;

public class Elevator extends SubsystemBase {
  private static Elevator instance = null;

  public final SparkMax LMot;
  public final SparkMax RMot;

  private final AbsoluteEncoder LeftEncoder;
  private final AbsoluteEncoder RightEncoder;

  public final Constraints ElevConstraints = new Constraints(Constants.Constraints.elevatorMaxVelocity, Constants.Constraints.elevatorMaxAcceleration);
  // Two PIDs so I and D can be consistant per side. 
  private final ProfiledPIDController pidLeft;
  private final ProfiledPIDController pidRight;


  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable ElevatorTable;
  private final NetworkTable ElevatorPIDs;
  private double target;
  private double enc_offset;

  private final SparkMaxConfig lConfig;
  private final SparkMaxConfig rConfig;

  private double speed;

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

    LMot.configure(lConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RMot.configure(rConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    LeftEncoder = new AbsoluteEncoder(Constants.SensorIDs.ElevEncoderLeft, Constants.Bot.leftElevatorBaseAngle);
    RightEncoder = new AbsoluteEncoder(Constants.SensorIDs.ElevEncoderRight, Constants.Bot.rightElevatorBaseAngle);


    pidLeft = new ProfiledPIDController(
        ElevatorPIDs.getEntry("P").getDouble(0),
        ElevatorPIDs.getEntry("I").getDouble(0),
        ElevatorPIDs.getEntry("D").getDouble(0),
        ElevConstraints);
    pidRight = new ProfiledPIDController(
        ElevatorPIDs.getEntry("P").getDouble(0),
        ElevatorPIDs.getEntry("I").getDouble(0),
        ElevatorPIDs.getEntry("D").getDouble(0),
        ElevConstraints);

    ElevatorPIDs.getEntry("P").setPersistent();
    ElevatorPIDs.getEntry("I").setPersistent();
    ElevatorPIDs.getEntry("D").setPersistent();
  }

  public void zero() {
    enc_offset = Enc.getPosition();
  }

  private double calculateSpeed() {
    return speed;
  } 

  private double getEncoderAverage() {
    return (LeftEncoder.get() + RightEncoder.get()) / 2;
  }

  @Override
  public void periodic() {
    pidLeft.setP(ElevatorPIDs.getEntry("P").getDouble(0));
    pidLeft.setI(ElevatorPIDs.getEntry("I").getDouble(0));
    pidLeft.setD(ElevatorPIDs.getEntry("D").getDouble(0));

    pidRight.setP(ElevatorPIDs.getEntry("P").getDouble(0));
    pidRight.setI(ElevatorPIDs.getEntry("I").getDouble(0));
    pidRight.setD(ElevatorPIDs.getEntry("D").getDouble(0));


    ElevatorTable.getEntry("Current Height").setDouble(getHeight());
    ElevatorTable.getEntry("Target Height").setDouble(target);
    speed = pidLeft.calculate(LeftEncoder.get(), new TrapezoidProfile.State(target, 0));
    if(Controller.getInstance().getControlMode() == Controller.ControlMode.MANUAL) return;
    LMot.set(speed);
    RMot.set(pidRight.calculate(RightEncoder.get(), new TrapezoidProfile.State(target, 0)));
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
}