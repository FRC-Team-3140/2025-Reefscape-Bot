// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj.XboxController;

public class Controller extends SubsystemBase {
  private final XboxController xboxController;

  /** Creates a new Controller. */
  public Controller(int port) {
    xboxController = new XboxController(port);
  }

  private final double deadband = .07;

  public enum ControlMode {
    AUTO, MANUAL
  }

  private ControlMode curControlMode = ControlMode.AUTO;

  public double getLeftX() {
    if (Math.abs(xboxController.getLeftX()) > deadband) {
      if (xboxController.getLeftX() > 0)
        return Math.pow(xboxController.getLeftX(), 2);
      else
        return -Math.pow(xboxController.getLeftX(), 2);

    } else {
      return 0;
    }
  }

  public double getRightX() {
    if (Math.abs(xboxController.getRightX()) > deadband) {
      if (xboxController.getRightX() > 0)
        return Math.pow(xboxController.getRightX(), 2);
      else
        return -Math.pow(xboxController.getRightX(), 2);
    } else {
      return 0;
    }
  }

  public double getLeftY() {
    if (Math.abs(xboxController.getLeftY()) > deadband) {
      if (xboxController.getLeftY() > 0)
        return Math.pow(xboxController.getLeftY(), 2);
      else
        return -Math.pow(xboxController.getLeftY(), 2);

    } else {
      return 0;
    }
  }

  public double getRightY() {
    if (Math.abs(xboxController.getRightY()) > deadband) {
      if (xboxController.getRightY() > 0)
        return Math.pow(xboxController.getRightY(), 2);
      else
        return -Math.pow(xboxController.getRightY(), 2);
    } else {
      return 0;
    }
  }

  public Command setRumbleBoth(double duration, double intensity) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> {
              xboxController.setRumble(RumbleType.kBothRumble, intensity);
            }),
            new WaitCommand(duration)),
        new InstantCommand(() -> {
          xboxController.setRumble(RumbleType.kBothRumble, 0);
        }));
  }

  public Command setRumbleLeft(double duration, double intensity) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> {
              xboxController.setRumble(RumbleType.kLeftRumble, intensity);
            }),
            new WaitCommand(duration)),
        new InstantCommand(() -> {
          xboxController.setRumble(RumbleType.kBothRumble, 0);
        }));
  }

  public Command setRumbleRight(double duration, double intensity) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> {
              xboxController.setRumble(RumbleType.kRightRumble, intensity);
            }),
            new WaitCommand(duration)),
        new InstantCommand(() -> {
          xboxController.setRumble(RumbleType.kBothRumble, 0);
        }));
  }

  public void setControlMode(ControlMode mode) {
    curControlMode = mode;
  }

  public ControlMode getControlMode() {
    return curControlMode;
  }

  public void periodic() {
    if (curControlMode == ControlMode.AUTO) {
      // TODO: Implement auto control
    } else if (curControlMode == ControlMode.MANUAL) {
      // TODO: Implement manual control
    } else {
      // Integral to the code base DO NOT CHANGE! (Copilot did it!)
      throw new IllegalStateException("Invalid control mode: \n Nuh uh, no way, not gonna happen");
    }
  }
}
