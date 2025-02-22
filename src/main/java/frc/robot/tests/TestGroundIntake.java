// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tests;

import frc.robot.subsystems.TestRunner.TestType;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.GroundIntake;

/** Add your docs here. */
public class TestGroundIntake extends Test {
    private final GroundIntake groundIntake = GroundIntake.getInstance();

    private final double angle = 90;

    public TestGroundIntake(NetworkTableEntry entry, TestType type) {
        super(entry, type);
    }

    @Override
    public void Start() {
        super.Start();
    }

    public void Periodic() {
        groundIntake.setAngle(angle);
        groundIntake.intake();
        System.out.println("Intaking Off Ground");
    }

    @Override
    public void Stop() {
        super.Stop();

        groundIntake.stopIntake();
    }
}
