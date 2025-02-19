// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tests;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.EndEffector;

/** Add your docs here. */
public class TestEndEffector extends Test {
    private final EndEffector endEffector = EndEffector.getInstance();

    private final double voltage = 0.25 * 12;

    public TestEndEffector(NetworkTableEntry entry) {
        super(entry);
    }

    @Override
    public void Start() {
        super.Start();
    }

    public void Periodic() {
        endEffector.setManipulatorVoltage(voltage);
    }

    @Override
    public void Stop() {
        super.Stop();
    }
}
