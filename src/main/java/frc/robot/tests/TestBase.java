// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tests;

/** An interface for all tests that can be run via the dev dashboard */
public interface TestBase {
    public void Start();
    public void Stop();
    public void Periodic();
}
