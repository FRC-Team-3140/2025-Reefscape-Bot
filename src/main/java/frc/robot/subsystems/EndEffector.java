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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class EndEffector extends SubsystemBase {

  static EndEffector instance = null;

  SparkMax beltMotorMN;
  SparkMax leftManipulatorMotorMN;
  SparkMax rightManipulatorMotorMN;

  SparkMax algaeIntakeMotorN;
  SparkMax algaeIntakeRotateMotorN;

  SparkMaxConfig breakModeConfig = new SparkMaxConfig();

  DutyCycleEncoder AlgaeArmEncoder;

  PIDController AlgaeArmPID;
  final double P = 0.000;
  final double I = 0.000;
  final double D = 0.000;

  double targetAngle = 0;

  double armAngle = 0;

  DigitalInput coralSensor = new DigitalInput(Constants.SensorIDs.EECoralSensor);

  public static EndEffector getInstance() {
    if (instance == null) {
      instance = new EndEffector();
    }
    return instance;
  }

  public EndEffector() {
    AlgaeArmPID = new PIDController(P, I, D);

    breakModeConfig.idleMode(IdleMode.kBrake);

    AlgaeArmEncoder = new DutyCycleEncoder(Constants.SensorIDs.AIEncoder);

    beltMotorMN = new SparkMax(Constants.MotorIDs.EETop, MotorType.kBrushless);
    leftManipulatorMotorMN = new SparkMax(Constants.MotorIDs.EELeft, MotorType.kBrushless);
    rightManipulatorMotorMN = new SparkMax(Constants.MotorIDs.EERight, MotorType.kBrushless);

    algaeIntakeMotorN = new SparkMax(Constants.MotorIDs.AIIntake, MotorType.kBrushless);
    algaeIntakeRotateMotorN = new SparkMax(Constants.MotorIDs.AIIntake, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(true).idleMode(IdleMode.kBrake).follow(Constants.MotorIDs.EERight);
    leftManipulatorMotorMN.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightManipulatorMotorMN.configure(breakModeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeIntakeMotorN.configure(breakModeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeIntakeRotateMotorN.configure(breakModeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    armAngle = AlgaeArmEncoder.get();
    algaeIntakeRotateMotorN.set(AlgaeArmPID.calculate(armAngle, targetAngle));
  }

  public void setAlgaeIntakeAngle(double angle) {
    targetAngle = angle;
  }

  public void setManipulatorSpeed(double speed) {
    rightManipulatorMotorMN.set(speed);
  }
  
  public void setBeltSpeed(double speed) {
    beltMotorMN.set(speed);
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