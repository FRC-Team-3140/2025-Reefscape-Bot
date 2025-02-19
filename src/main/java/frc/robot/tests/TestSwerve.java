// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tests;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class TestSwerve extends Test {
    private final SwerveDrive swerve = SwerveDrive.getInstance();
    
    private long lastSwitchTime = System.currentTimeMillis();
    private int stage = 0;

    public TestSwerve(NetworkTableEntry entry) {
        super(entry);
    }

    @Override
    public void Start() {
        super.Start();
    }


    public void Periodic() {
        // Toggles between straight driving, horizontal driving, and turning
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastSwitchTime >= 2000) {
            stage = (stage + 1) % 3;
            lastSwitchTime = currentTime;
        }

        switch (stage) {
            case 0:
                // First section of code
                swerve.drive(Constants.Bot.maxChassisSpeed * 0.5, 0, 0, false);
                break;
            case 1:
                // Second section of code
                swerve.drive(0, Constants.Bot.maxChassisSpeed * 0.5, 0, false);
                break;
            case 2:
                // Third section of code
                swerve.drive(0, 0, Constants.Bot.maxTurnSpeed * 0.5, false);
                break;
        }
    }

    @Override
    public void Stop() {
        super.Stop();
    }
}
