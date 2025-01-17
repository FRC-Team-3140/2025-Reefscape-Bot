// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
 * This subsystem will manage the sensors that are 
 * used to align the robot with the reef. The sensors 
 * will be placed so that when sensors 1-3 detect the 
 * reef close to them, the robot will be aligned with 
 * the reef's left pipe. When sensors 0-2 detect the 
 * reef close to them, the robot will be aligned with 
 * the reef's right pipe. When all sensors detect the 
 * reef, or 1-2 detect the reef, the robot will be 
 * able to self align.
 * 
 * The current sensors are undecided, but distance 
 * sensors of some kind would be better than plain 
 * object detectors to allow for 2-axis alignment.
 */
public class ReefAlignment extends SubsystemBase {
  
  
  AnalogInput[] SensorArray = new AnalogInput[4]; 
  

  public ReefAlignment() {
    for(int i = 0; i < SensorArray.length; i++) {
      SensorArray[i] = new AnalogInput(Constants.SensorIDs.distanceSensorArray[i]);
    }
  }

  @Override
  public void periodic() {
    //SensorArray[0].getValue();
  }
}
