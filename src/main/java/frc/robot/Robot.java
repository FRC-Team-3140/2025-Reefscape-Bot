// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.test.TestRunner;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private TestRunner testCommand = null;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = RobotContainer.getInstance();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (testCommand != null) {
      testCommand.cancel();
      testCommand = null;
    }
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (testCommand != null) {
      testCommand.cancel();
      testCommand = null;
    }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (testCommand != null) {
      testCommand.cancel();
      testCommand = null;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    try {
      Constants.PathplannerConstants.config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    if (testCommand != null) {
      testCommand.cancel();
      testCommand = null;
    }
    testCommand = new TestRunner();
    CommandScheduler.getInstance().schedule(testCommand);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // dashboard stuff
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Dashboard");
    NetworkTable devBoard = table.getSubTable("Dev");
    NetworkTableEntry swerveButton = devBoard.getEntry("Swerve");
    NetworkTableEntry algaeButton = devBoard.getEntry("AlgaeIntake");
    NetworkTableEntry effectorButton = devBoard.getEntry("End Effector");
    NetworkTableEntry groundButton = devBoard.getEntry("Ground Intake");
    NetworkTableEntry elevatorButton = devBoard.getEntry("Elevator");

    if (testCommand != null) {
      testCommand.setRunning(TestRunner.TestType.SWERVE, swerveButton.getBoolean(false));
      testCommand.setRunning(TestRunner.TestType.ALGAE_INTAKE, algaeButton.getBoolean(false));
      testCommand.setRunning(TestRunner.TestType.END_EFFECTOR, effectorButton.getBoolean(false));
      testCommand.setRunning(TestRunner.TestType.GROUND_INTAKE, groundButton.getBoolean(false));
      testCommand.setRunning(TestRunner.TestType.ELEVATOR, elevatorButton.getBoolean(false));
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
