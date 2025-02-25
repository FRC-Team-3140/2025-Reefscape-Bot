// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.swerveDrive.SwerveDriveManualControl;
import frc.robot.libs.LoggedCommand;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public SwerveDrive swerveDrive = SwerveDrive.getInstance();
  public Elevator elevator = Elevator.getInstance();
  public EndEffector endEffector = EndEffector.getInstance();
  public GroundIntake groundIntake = GroundIntake.getInstance();
  public TestRunner testRunner = TestRunner.getInstance();
  public Controller controller = Controller.getInstance();

  private static RobotContainer container = null;

  // Get the singleton instance or create it if it doesn't exist
  public static RobotContainer getInstance() {
    if (container == null) {
      container = new RobotContainer();
    }
    return container;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    // TODO: figure out where to put this for final version V
    swerveDrive.setDefaultCommand(new SwerveDriveManualControl(swerveDrive, Constants.Bot.maxChassisSpeed, Constants.Bot.maxChassisTurnSpeed));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public LoggedCommand getAutonomousCommand() {
    return null;
  }
}