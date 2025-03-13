// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tests;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.TestRunner;
import frc.robot.subsystems.TestRunner.TestType;

/** An interface for all tests that can be run via the dev dashboard */
public class Test {
    public boolean running = false;
    public NetworkTableEntry ntEntry;
    public TestType type;

    public Test(NetworkTableEntry ntEntry, TestType type) {
        this.ntEntry = ntEntry;
        this.type = type;
        ntEntry.setBoolean(false);
    }

    public void Start() {
        System.out.println("Starting test: " + this.getClass().getName());
        running = true;
        ntEntry.setBoolean(true);
    }

    public void Stop() {
        System.out.println("Stopping test: " + this.getClass().getName());
        running = false;
        ntEntry.setBoolean(false);
    }

    public void Periodic() {
    }

    public void QueryNetworkTable() {
        if (ntEntry.getBoolean(false) != running) {
            System.out.println("Printing from class: " + this.getClass().getName());
            TestRunner.getInstance().setState(type, !running);
        }
    }
}
