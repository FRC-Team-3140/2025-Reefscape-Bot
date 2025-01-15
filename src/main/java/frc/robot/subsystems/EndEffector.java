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

public class EndEffector extends SubsystemBase {

  SparkMax beltMotorMN;
  SparkMax leftManipulatorMotorMN;
  SparkMax rightManipulatorMotorMN;

  SparkMax algaeIntakeMotorN;
  SparkMax algaeIntakeRotateMotorN;

  SparkMaxConfig breakModeConfig = new SparkMaxConfig();
  SparkMaxConfig coastModeConfig = new SparkMaxConfig();

  DutyCycleEncoder AlgaeArmEncoder;

  PIDController AlgaeArmPID;
  final double P = 0.000;
  final double I = 0.000;
  final double D = 0.000;

  double targetAngle = 0;

  public EndEffector() {
    AlgaeArmPID = new PIDController(P, I, D);

    breakModeConfig.idleMode(IdleMode.kBrake);
    coastModeConfig.idleMode(IdleMode.kCoast);

    AlgaeArmEncoder = new DutyCycleEncoder(Constants.EncoderID.AIEncoder);

    beltMotorMN = new SparkMax(Constants.MotorID.EETop, MotorType.kBrushless);
    leftManipulatorMotorMN = new SparkMax(Constants.MotorID.EELeft, MotorType.kBrushless);
    rightManipulatorMotorMN = new SparkMax(Constants.MotorID.EERight, MotorType.kBrushless);

    algaeIntakeMotorN = new SparkMax(Constants.MotorID.AIIntake, MotorType.kBrushless);
    algaeIntakeRotateMotorN = new SparkMax(Constants.MotorID.AIIntake, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(true).idleMode(IdleMode.kCoast).follow(Constants.MotorID.EERight);
    leftManipulatorMotorMN.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightManipulatorMotorMN.configure(coastModeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeIntakeMotorN.configure(coastModeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeIntakeRotateMotorN.configure(breakModeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  @Override
  public void periodic() {
    AlgaeArmPID.setSetpoint(targetAngle);
    algaeIntakeRotateMotorN.setVoltage(AlgaeArmPID.calculate(AlgaeArmEncoder.get()));
  }
  
}
