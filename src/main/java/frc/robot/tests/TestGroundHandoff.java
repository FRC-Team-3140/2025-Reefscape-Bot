// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tests;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/** Add your docs here. */
public class TestGroundHandoff extends Test {
    public TestGroundHandoff(NetworkTableEntry entry) {
        super(entry);
    }

    @Override
    public void Start() {
        super.Start();
    }

    public void Periodic() {
        // TODO: Implement this method
        new PrintCommand("Ground Handoff").schedule();
    }

    @Override
    public void Stop() {
        super.Stop();
    }
}
