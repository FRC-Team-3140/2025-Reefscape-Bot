// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.tests.TestAlgaeIntake;
import frc.robot.tests.TestAlgaeReef;
import frc.robot.tests.TestBase;
import frc.robot.tests.TestElevator;
import frc.robot.tests.TestEndEffector;
import frc.robot.tests.TestGroundHandoff;
import frc.robot.tests.TestGroundIntake;
import frc.robot.tests.TestSwerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestRunner extends Command {
  public enum TestType {
    SWERVE,
    ALGAE_INTAKE,
    END_EFFECTOR,
    GROUND_INTAKE,
    ELEVATOR,
    GROUND_HANDOFF,
    SOURCE_HANDOFF,
    ALGAE_REEF,
    ALGAE_GROUND
  };

  private final HashMap<TestType, Boolean> runningTests = new HashMap<TestType, Boolean>(5);
  private final HashMap<TestType, TestBase> tests = new HashMap<TestType, TestBase>(5);

  /** Creates a new TestRunner. */
  public TestRunner() {
    runningTests.put(TestType.SWERVE, false);
    runningTests.put(TestType.ALGAE_INTAKE, false);
    runningTests.put(TestType.END_EFFECTOR, false);
    runningTests.put(TestType.GROUND_INTAKE, false);
    runningTests.put(TestType.ELEVATOR, false);
    runningTests.put(TestType.GROUND_HANDOFF, false);
    runningTests.put(TestType.GROUND_INTAKE, false);
    runningTests.put(TestType.ALGAE_REEF, false);
    runningTests.put(TestType.ALGAE_GROUND, false);

    tests.put(TestType.SWERVE, new TestSwerve());
    tests.put(TestType.ALGAE_INTAKE, new TestAlgaeIntake());
    tests.put(TestType.END_EFFECTOR, new TestEndEffector());
    tests.put(TestType.GROUND_INTAKE, new TestGroundIntake());
    tests.put(TestType.ELEVATOR, new TestElevator());
    tests.put(TestType.GROUND_HANDOFF, new TestGroundHandoff());
    tests.put(TestType.GROUND_INTAKE, new TestGroundIntake());
    tests.put(TestType.ALGAE_REEF, new TestAlgaeReef());
    tests.put(TestType.ALGAE_INTAKE, new TestAlgaeIntake());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (TestType type : tests.keySet()) { // loop through every test types
      if (!runningTests.get(type)) { // skip it if its not running
        continue;
      }

      // periodic if its running
      tests.get(type).Periodic();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (TestType type : tests.keySet()) { // loop through every test types
      if (!runningTests.get(type)) { // skip it if its not running
        continue;
      }

      // disable it
      runningTests.put(type, false);
      tests.get(type).Stop();
    }
  }

  public boolean isRunning(TestType type) {
    return runningTests.get(type);
  }

  public void setRunning(TestType type, boolean running) {
    if (runningTests.get(type) == running)
      return; // skip because the value is already correct
    
    runningTests.put(type, running);
    if (running) {
      tests.get(type).Start();
    } else {
      tests.get(type).Stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
