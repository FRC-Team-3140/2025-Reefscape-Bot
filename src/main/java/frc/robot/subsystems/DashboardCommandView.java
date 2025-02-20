// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DashboardCommandView extends SubsystemBase {

  private static DashboardCommandView instance = null;
  public static DashboardCommandView getInstance() {
    if (instance == null) {
      instance = new DashboardCommandView();
    }
    return instance;
  }

  /** Creates a new DashboardCommandView. */
  public DashboardCommandView() {
    CommandScheduler.getInstance().onCommandInitialize((command) -> {
      System.out.println("Command " + command.getName() + " started");
    });
    CommandScheduler.getInstance().onCommandExecute((command) -> {
      //System.out.println("Command " + command.getName() + " executing");
    });
    CommandScheduler.getInstance().onCommandFinish((command) -> {
      System.out.println("Command " + command.getName() + " ended");
    });
    CommandScheduler.getInstance().onCommandInterrupt((command) -> {
      System.out.println("Command " + command.getName() + " interrupted");
    });
  
  }

  @Override
  public void periodic() {
    
  }
}
