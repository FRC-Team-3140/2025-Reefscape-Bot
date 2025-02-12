// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tests;

import edu.wpi.first.networktables.NetworkTableEntry;

/** An interface for all tests that can be run via the dev dashboard */
public class Test {
    public boolean running = false;
    public NetworkTableEntry ntEntry;

    public Test(NetworkTableEntry ntEntry) {
        this.ntEntry = ntEntry;
        ntEntry.setBoolean(false);
    }

    public void Start() {
        running = true;
        ntEntry.setBoolean(true);
    }

    public void Stop() {
        running = false;
        ntEntry.setBoolean(false);
    }

    public void Periodic() {
    }
}
