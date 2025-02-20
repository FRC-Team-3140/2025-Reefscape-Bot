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
import frc.robot.commands.GroundCoralIntake;
import frc.robot.commands.IntakeAlgaeReef;
import frc.robot.commands.SourceCoralIntake;
import frc.robot.libs.NetworkTables;
import edu.wpi.first.wpilibj.XboxController;

public class Controller extends SubsystemBase {
  private static Controller instance = null;

  public final XboxController primaryController;
  public final XboxController secondaryController;

  private final Elevator elevator = Elevator.getInstance();
  private final EndEffector endEffector = EndEffector.getInstance();

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

  private ControlMode curControlMode = ControlMode.MANUAL; // Default to auto when auto is implemented

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
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      if (secondaryController.getRightBumperButtonPressed()) {
        curControlMode = ControlMode.MANUAL;
        System.out.println("Control Mode: " + curControlMode);
      } else if (secondaryController.getLeftBumperButtonPressed()) {
        curControlMode = ControlMode.AUTO;
        System.out.println("Control Mode: " + curControlMode);
      }

      return;
    }

    if (primaryController.getYButtonPressed()) {
      // TODO: Look at resetGyro() in Odometry.java
      SwerveDrive.odometry.resetGyro();
    }
  }

  private void manualControlMode() {
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      if (secondaryController.getRightBumperButtonPressed()) {
        curControlMode = ControlMode.MANUAL;
        System.out.println("Control Mode: " + curControlMode);
      } else if (secondaryController.getLeftBumperButtonPressed()) {
        curControlMode = ControlMode.AUTO;
        System.out.println("Control Mode: " + curControlMode);
      }
      

      return;
    }

    if (secondaryController.getBButtonPressed()) {
      // Elevator trough
      elevator.setHeight(ElevatorHeights.reefCoralL1Height);
    }

    if (secondaryController.getAButtonPressed()) {
      // Elevator level reef 1
      elevator.setHeight(ElevatorHeights.reefCoralL2Height);
    }

    if (secondaryController.getXButtonPressed()) {
      // Elevator level reef 2
      elevator.setHeight(ElevatorHeights.reefCoralL3Height);
    }

    if (secondaryController.getYButtonPressed()) {
      // Elevator level reef 3
      elevator.setHeight(ElevatorHeights.reefCoralL4Height);
    }

    if (secondaryController.getStartButtonPressed()) {
      // Get Algae L2
      new IntakeAlgaeReef(endEffector, ElevatorHeights.reefAlgaeL1Height).schedule();;
    }

    if (secondaryController.getBackButtonPressed()) {
      // Get Algae L3
      new IntakeAlgaeReef(endEffector, ElevatorHeights.reefAlgaeL2Height).schedule();;
    }

    if (secondaryController.getRightTriggerAxis() > Constants.Controller.triggerThreshold) {
      // Score coral
      endEffector.setManipulatorSpeed(Constants.MotorSpeeds.EndEffector.manipulatorScore);
    } else {
      endEffector.setManipulatorSpeed(0);
    }

    if (secondaryController.getLeftBumperButtonPressed()) {
      // Ground Intake
      new GroundCoralIntake(GroundIntake.getInstance(), Elevator.getInstance()).schedule();
    }

    if (secondaryController.getRightBumperButtonPressed()) {
      // Source Intake
      new SourceCoralIntake().schedule();
    } 

    if (secondaryController.getPOV() == 180) {
      // Stow elevator
      elevator.setHeight(ElevatorHeights.safeStowed);
    }

    if (secondaryController.getLeftTriggerAxis() > Constants.Controller.triggerThreshold) {
      // Intake Algae
      endEffector.setAlgaeIntakeSpeed(Constants.MotorSpeeds.EndEffector.algaeProcessorSpeed);
      endEffector.setAlgaeIntakeAngle(Constants.AlgaeIntakeAngles.processorScoreTop);
    } else {
      endEffector.setAlgaeIntakeSpeed(0);
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
