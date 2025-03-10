// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tests;

import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TestRunner.TestType;
import edu.wpi.first.networktables.NetworkTableEntry;

/** Add your docs here. */
public class TestElevator extends Test {
    private final Elevator elevator = Elevator.getInstance();

    private long lastSwitchTime = System.currentTimeMillis();
    private int stage = 0;

    public TestElevator(NetworkTableEntry entry, TestType type) {
        super(entry, type);
    }

    @Override
    public void Start() {
        super.Start();
    }

    public void Periodic() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastSwitchTime >= 2000) {
            stage = (stage + 1) % 2;
            lastSwitchTime = currentTime;
        }

        switch (stage) {
            case 0:
                // First section of code
                elevator.setHeight(Constants.ElevatorHeights.maxiumum);
                break;
            case 1:
                // Second section of code
                elevator.setHeight(Constants.ElevatorHeights.minimum);
                break;
        }
    }

    @Override
    public void Stop() {
        super.Stop();
    }
}
