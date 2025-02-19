// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class NetworkTables {
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  
  private static NetworkTable dash = inst.getTable(Constants.NetworktablePaths.Dashboard);
  
  private static NetworkTable dsInfo = dash.getSubTable(Constants.NetworktablePaths.DS);
    public static NetworkTableEntry state_s = dsInfo.getEntry("state_s");
    public static NetworkTableEntry voltage_d = dsInfo.getEntry("voltage_d");

  private static NetworkTable reef = dash.getSubTable(Constants.NetworktablePaths.Reef);
    public static NetworkTableEntry loc_s = reef.getEntry("loc_s");
    public static NetworkTableEntry algae_b = reef.getEntry("algae_b");
    public static NetworkTableEntry autoRunning_b = reef.getEntry("autoRunning_b");

  private static NetworkTable devBoard = dash.getSubTable(Constants.NetworktablePaths.Test);
    public static NetworkTableEntry swerveButton_b = devBoard.getEntry("Swerve_b");
    public static NetworkTableEntry algaeButton_b = devBoard.getEntry("Algae Intake_b");
    public static NetworkTableEntry effectorButton_b = devBoard.getEntry("End Effector_b");
    public static NetworkTableEntry groundButton_b = devBoard.getEntry("Ground Intake_b");
    public static NetworkTableEntry elevatorButton_b = devBoard.getEntry("Elevator");
    public static NetworkTableEntry handoffButton_b = devBoard.getEntry("Ground Handoff_b");
    public static NetworkTableEntry sourceButton_b = devBoard.getEntry("Source Handoff_b");
    public static NetworkTableEntry reefButton_b = devBoard.getEntry("Algae Reef_b");
    public static NetworkTableEntry algaeGroundButton_b = devBoard.getEntry("Algae Ground_b");
  
  private static NetworkTable misc = dash.getSubTable(Constants.NetworktablePaths.Misc);
    public static NetworkTableEntry driveModeManual_b = misc.getEntry("driveModeManual_b");
}
  