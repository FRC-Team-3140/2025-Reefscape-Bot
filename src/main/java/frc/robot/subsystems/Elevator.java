// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private static Elevator instance = null;

  private final SparkMax LMot;
  private final SparkMax RMot;
  private final DutyCycleEncoder Enc;
  private final ProfiledPIDController pid;
  private final NetworkTable ElevatorPIDs;
  private double target;

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  /** Creates a new Elevator. */
  private Elevator() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable PIDsTable = inst.getTable("PIDs");
    ElevatorPIDs = PIDsTable.getSubTable("Elevator");

    LMot = new SparkMax(Constants.MotorIDs.ElevLNeo, MotorType.kBrushless);
    RMot = new SparkMax(Constants.MotorIDs.ElevRNeo, MotorType.kBrushless);

    Enc = new DutyCycleEncoder(Constants.SensorIDs.ElevEncoder);

    pid = new ProfiledPIDController(
        ElevatorPIDs.getEntry("P").getDouble(0),
        ElevatorPIDs.getEntry("I").getDouble(0),
        ElevatorPIDs.getEntry("D").getDouble(0),
        Constants.Constraints.ElevConstraints);

    ElevatorPIDs.getEntry("P").setPersistent();
    ElevatorPIDs.getEntry("I").setPersistent();
    ElevatorPIDs.getEntry("D").setPersistent();
  }

  private double calculateSpeed() {
    return pid.calculate(Enc.get(), new TrapezoidProfile.State(target, 0));
  }

  @Override
  public void periodic() {
    pid.setP(ElevatorPIDs.getEntry("P").getDouble(0));
    pid.setI(ElevatorPIDs.getEntry("I").getDouble(0));
    pid.setD(ElevatorPIDs.getEntry("D").getDouble(0));

    double speed = calculateSpeed();
    LMot.set(speed);
    RMot.set(-speed);
  }

  public void setHeight(double height) {
    target = height; // TODO: DO MATH
  }

  public boolean isMoving() {
    return Math.abs(calculateSpeed()) > Constants.Limits.ElevMovement;
  }
}