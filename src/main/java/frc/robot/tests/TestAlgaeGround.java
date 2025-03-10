// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tests;

import frc.robot.commands.endeffector.EndEffectorIntakeAlgae;
import frc.robot.subsystems.TestRunner.TestType;
import edu.wpi.first.networktables.NetworkTableEntry;

/** Add your docs here. */
public class TestAlgaeGround extends Test {
    private final EndEffectorIntakeAlgae intakeCommand = new EndEffectorIntakeAlgae(
            EndEffectorIntakeAlgae.Level.Ground);

    public TestAlgaeGround(NetworkTableEntry entry, TestType type) {
        super(entry, type);
    }

    @Override
    public void Start() {
        super.Start();
    }

    public void Periodic() {
        if (!intakeCommand.isScheduled()) {
            intakeCommand.schedule();
        }
    }

    @Override
    public void Stop() {
        super.Stop();
    }
}
