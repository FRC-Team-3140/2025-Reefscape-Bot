// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tests;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.GroundIntake;

/** Add your docs here. */
public class TestGroundIntake extends Test {
    private final GroundIntake groundIntake = GroundIntake.getInstance();

    private final double angle = 90;

    public TestGroundIntake(NetworkTableEntry entry) {
        super(entry);
    }

    @Override
    public void Start() {
        super.Start();
    }

    public void Periodic() {
        groundIntake.setAngle(angle);
        groundIntake.intake();
    }

    @Override
    public void Stop() {
        super.Stop();

        groundIntake.stopIntake();
    }
}
