// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeights;

import edu.wpi.first.wpilibj.XboxController;

public class Controller extends SubsystemBase {
  private static Controller instance = null;

  public final XboxController primaryController;
  public final XboxController secondaryController;

  private final SwerveDrive serve = SwerveDrive.getInstance();
  private final Elevator elevator = Elevator.getInstance();
  private final EndEffector endEffector = EndEffector.getInstance();
  private final GroundIntake groundIntake = GroundIntake.getInstance();

  /** Creates a new Controller. */
  public Controller(int primary, int secondary) {
    primaryController = new XboxController(primary);
    secondaryController = new XboxController(secondary);
  }

  private final double deadband = .07;

  public enum controllers {
    PRIMARY, SECONDARY
  }

  public enum ControlMode {
    AUTO, MANUAL
  }

  private ControlMode curControlMode = ControlMode.AUTO;

  public static Controller getInstance() {
    if (instance == null) {
      instance = new Controller(Constants.Controller.DriverControllerPort,
          Constants.Controller.SecondaryDriverControllerPort);
    }
    return instance;
  }

  public double getLeftX(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    if (Math.abs(contr.getLeftX()) > deadband) {
      if (contr.getLeftX() > 0)
        return Math.pow(contr.getLeftX(), 2);
      else
        return -Math.pow(contr.getLeftX(), 2);

    } else {
      return 0;
    }
  }

  public double getRightX(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    if (Math.abs(contr.getRightX()) > deadband) {
      if (contr.getRightX() > 0)
        return Math.pow(contr.getRightX(), 2);
      else
        return -Math.pow(contr.getRightX(), 2);
    } else {
      return 0;
    }
  }

  public double getLeftY(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    if (Math.abs(contr.getLeftY()) > deadband) {
      if (contr.getLeftY() > 0)
        return Math.pow(contr.getLeftY(), 2);
      else
        return -Math.pow(contr.getLeftY(), 2);

    } else {
      return 0;
    }
  }

  public double getRightY(controllers controller) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    if (Math.abs(contr.getRightY()) > deadband) {
      if (contr.getRightY() > 0)
        return Math.pow(contr.getRightY(), 2);
      else
        return -Math.pow(contr.getRightY(), 2);
    } else {
      return 0;
    }
  }

  public void setRumbleBoth(controllers controller, double duration, double intensity) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> {
              contr.setRumble(RumbleType.kBothRumble, intensity);
            }),
            new WaitCommand(duration)),
        new InstantCommand(() -> {
          contr.setRumble(RumbleType.kBothRumble, 0);
        })).schedule();
  }

  public void setRumbleLeft(controllers controller, double duration, double intensity) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> {
              contr.setRumble(RumbleType.kLeftRumble, intensity);
            }),
            new WaitCommand(duration)),
        new InstantCommand(() -> {
          contr.setRumble(RumbleType.kBothRumble, 0);
        })).schedule();
  }

  public void setRumbleRight(controllers controller, double duration, double intensity) {
    XboxController contr = (controller == controllers.PRIMARY ? primaryController : secondaryController);
    new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> {
              contr.setRumble(RumbleType.kRightRumble, intensity);
            }),
            new WaitCommand(duration)),
        new InstantCommand(() -> {
          contr.setRumble(RumbleType.kBothRumble, 0);
        })).schedule();
  }

  public void setControlMode(ControlMode mode) {
    curControlMode = mode;
  }

  public ControlMode getControlMode() {
    return curControlMode;
  }

  private void autoControlMode() {
    if (primaryController.getYButton()) {
      // TODO: Remember how to do this
      serve.resetGyro();
    }

    // Secondary controller enable manual controll
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      if (secondaryController.getRightBumperButton()) {
        curControlMode = ControlMode.MANUAL;
      } else if (secondaryController.getLeftBumperButton()) {
        curControlMode = ControlMode.AUTO;
      }
    }
  }

  private void manualControlMode() {
    if (secondaryController.getBButton()) {
      // Elevator trough
      elevator.setHeight(ElevatorHeights.reefCoralL1Height);
    }

    if (secondaryController.getAButton()) {
      // Elevator level reef 1
      elevator.setHeight(ElevatorHeights.reefCoralL2Height);
    }

    if (secondaryController.getXButton()) {
      // Elevator level reef 2
      elevator.setHeight(ElevatorHeights.reefCoralL3Height);
    }

    if (secondaryController.getYButton()) {
      // Elevator level reef 3
      elevator.setHeight(ElevatorHeights.reefCoralL4Height);
    }

    if (secondaryController.getStartButton()) {
      // Get Algae L2
      elevator.setHeight(ElevatorHeights.reefAlgaeL1Height);
    }

    if (secondaryController.getBButton()) {
      // Get Algae L3
      elevator.setHeight(ElevatorHeights.reefAlgaeL2Height);
    }

    if (secondaryController.getRightTriggerAxis() > Constants.Controller.triggerThreshold) {
      // Score coral
      endEffector.setManipulatorVoltage(Constants.Bot.coralManipulatorVoltage);
    } else {
      endEffector.setManipulatorVoltage(0);
    }

    if (secondaryController.getLeftBumperButton()) {
      // Ground Intake
      groundIntake.intake();
    }

    if (secondaryController.getRightBumperButton()) {
      // Source Intake
      elevator.setHeight(ElevatorHeights.sourceIntake);
    }

    // Secondary controller disable manual controll
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      if (secondaryController.getRightBumperButton()) {
        curControlMode = ControlMode.MANUAL;
      } else if (secondaryController.getLeftBumperButton()) {
        curControlMode = ControlMode.AUTO;
      }
    }
  }

  public void periodic() {
    NetworkTables.driveModeManual_b.setBoolean(curControlMode == ControlMode.MANUAL);
    switch (curControlMode) {
      case AUTO:
        autoControlMode();
        break;
      case MANUAL:
        manualControlMode();
        break;
      default:
        // Integral to the code base DO NOT CHANGE! (Copilot did it!)
        throw new IllegalStateException("Invalid control mode: \n Nuh uh, no way, not gonna happen");
    }
  }
}
