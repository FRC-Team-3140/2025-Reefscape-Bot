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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class EndEffector extends SubsystemBase {

  static EndEffector instance = null;

  SparkMax leftManipulatorMotorMN;
  SparkMax rightManipulatorMotorMN;

  SparkMax algaeIntakeMotorN;
  SparkMax algaeIntakeRotateMotorN;

  DutyCycleEncoder AlgaeArmEncoder;

  ProfiledPIDController AlgaeArmPID;
  final double P = 2.25;
  final double I = 0;
  final double D = 0;

  double targetAngle = Constants.AlgaeIntakeAngles.max;

  double armAngle = 0;

  DigitalInput coralSensor = new DigitalInput(Constants.SensorIDs.EECoralSensor);

  public static EndEffector getInstance() {
    if (instance == null) {
      instance = new EndEffector();
    }
    return instance;
  }

  public EndEffector() {
    AlgaeArmPID = new ProfiledPIDController(P, I, D, new Constraints(10, 8));
    AlgaeArmEncoder = new DutyCycleEncoder(Constants.SensorIDs.AIEncoder, 1, 0.5);

    leftManipulatorMotorMN = new SparkMax(Constants.MotorIDs.EELeft, MotorType.kBrushless);
    rightManipulatorMotorMN = new SparkMax(Constants.MotorIDs.EERight, MotorType.kBrushless);

    algaeIntakeMotorN = new SparkMax(Constants.MotorIDs.AIIntake, MotorType.kBrushless);
    algaeIntakeRotateMotorN = new SparkMax(Constants.MotorIDs.AIRotate, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(false).idleMode(IdleMode.kBrake);
    leftManipulatorMotorMN.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(true).idleMode(IdleMode.kBrake);
    rightManipulatorMotorMN.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeIntakeMotorN.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeIntakeRotateMotorN.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    armAngle = AlgaeArmEncoder.get();
    NetworkTableInstance.getDefault().getEntry("Endeffector").setDouble(armAngle);
    AlgaeArmPID.setGoal(targetAngle);
    algaeIntakeRotateMotorN.set(AlgaeArmPID.calculate(armAngle, targetAngle));
  }

  public void setAlgaeIntakeAngle(double angle) {
    targetAngle = angle;
  }

  public void setManipulatorSpeed(double speed) {
    rightManipulatorMotorMN.set(Math.min(speed, 0.8));
    leftManipulatorMotorMN.set(Math.min(speed, 0.8));
  }

  public void setAlgaeIntakeSpeed(double speed) {
    algaeIntakeMotorN.set(speed);
  }

  public double getAlgaeIntakeAngle() {
    return armAngle;
  }

  public double getAlgaeIntakeCurrent() {
    return algaeIntakeMotorN.getOutputCurrent();
  }

  public boolean hasCoral() {
    return !coralSensor.get();
  }
}