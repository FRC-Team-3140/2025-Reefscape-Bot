// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tests;

import frc.robot.commands.compoundCommands.GetAlgaeReefManual;
import frc.robot.commands.endeffector.EndEffectorIntakeAlgae;
import frc.robot.subsystems.TestRunner.TestType;
import edu.wpi.first.networktables.NetworkTableEntry;

/** Add your docs here. */
public class TestAlgaeReef extends Test {
    private final GetAlgaeReefManual algaeCommand = new GetAlgaeReefManual(EndEffectorIntakeAlgae.Level.AlgaeL1);

    public TestAlgaeReef(NetworkTableEntry entry, TestType type) {
        super(entry, type);
    }

    @Override
    public void Start() {
        super.Start();
    }

    public void Periodic() {
        if (!algaeCommand.isScheduled()) {
            algaeCommand.schedule();
        }
    }

    @Override
    public void Stop() {
        super.Stop();
    }
}
