// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tests;
import frc.robot.subsystems.TestRunner.TestType;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.EndEffector;

/** Add your docs here. */
public class TestEndEffector extends Test {
    private final EndEffector endEffector = EndEffector.getInstance();

    private final double voltage = 0.25;

    public TestEndEffector(NetworkTableEntry entry, TestType type) {
        super(entry, type);
    }

    @Override
    public void Start() {
        super.Start();
    }

    public void Periodic() {
        endEffector.setManipulatorSpeed(voltage);
    }

    @Override
    public void Stop() {
        super.Stop();
    }
}
