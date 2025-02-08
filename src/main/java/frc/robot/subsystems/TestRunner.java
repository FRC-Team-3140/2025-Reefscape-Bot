// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.tests.TestAlgaeIntake;
import frc.robot.tests.TestAlgaeReef;
import frc.robot.tests.Test;
import frc.robot.tests.TestElevator;
import frc.robot.tests.TestEndEffector;
import frc.robot.tests.TestGroundHandoff;
import frc.robot.tests.TestGroundIntake;
import frc.robot.tests.TestSwerve;

// TODO: make a stop all command to run in telep and auto init
public class TestRunner extends SubsystemBase {
  private static TestRunner instance = null;

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

  private final HashMap<TestType, Test> tests = new HashMap<TestType, Test>();

  public static TestRunner getInstance() {
    if (instance == null) {
      instance = new TestRunner();
    }
    return instance;
  }

  /** Creates a new TestRunner. */
  private TestRunner() {

    tests.put(TestType.SWERVE, new TestSwerve(NetworkTables.swerveButton_b));
    tests.put(TestType.ALGAE_INTAKE, new TestAlgaeIntake(NetworkTables.algaeButton_b));
    tests.put(TestType.END_EFFECTOR, new TestEndEffector(NetworkTables.effectorButton_b));
    tests.put(TestType.GROUND_INTAKE, new TestGroundIntake(NetworkTables.groundButton_b));
    tests.put(TestType.ELEVATOR, new TestElevator(NetworkTables.elevatorButton_b));
    tests.put(TestType.GROUND_HANDOFF, new TestGroundHandoff(NetworkTables.handoffButton_b));
    tests.put(TestType.GROUND_INTAKE, new TestGroundIntake(NetworkTables.reefButton_b));
    tests.put(TestType.ALGAE_REEF, new TestAlgaeReef(NetworkTables.algaeButton_b));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (TestType type : tests.keySet()) { // loop through every test types
      if (!tests.get(type).running) { // skip it if its not running
        continue;
      }

      // periodic if its running
      tests.get(type).Periodic();
    }
  }

  public boolean isRunning(TestType type) {
    return tests.get(type).running;
  }

  public void setState(TestType type, boolean run) {
    if (tests.get(type).running == run)
      return;

    if (run) {
      tests.get(type).Start();
    } else {
      tests.get(type).Stop();
    }
  }

  public void updateStates() {
    for (TestType type : tests.keySet()) { // loop through every test types
      setState(type, tests.get(type).ntEntry.getBoolean(false));
    }
  }
}
