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
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorHeights;
import frc.robot.commands.GoToSourceAndIntake;
import frc.robot.commands.IntakeAlgaeReef;
import frc.robot.commands.SourceCoralIntake;
import frc.robot.commands.elevator.SetHeight;
import frc.robot.commands.endeffector.EndEffectorScoreCoral;
import frc.robot.commands.swerveDrive.SwerveDriveManualControl;
import frc.robot.libs.NetworkTables;
import edu.wpi.first.wpilibj.XboxController;

public class Controller extends SubsystemBase {
  private static Controller instance = null;

  public final XboxController primaryController;
  public final XboxController secondaryController;

  private final Elevator elevator = Elevator.getInstance();
  private final EndEffector endEffector = EndEffector.getInstance();

  private boolean fieldOriented = true;

  /** Creates a new Controller. */
  public Controller(int primary, int secondary) {
    primaryController = new XboxController(primary);
    secondaryController = new XboxController(secondary);
  }

  private final double deadband = .07;

  private final boolean testing = false;

  public enum controllers {
    PRIMARY, SECONDARY
  }

  public enum ControlMode {
    AUTO, MANUAL, OHNO_MANUAL
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

  private void updateControlMode() {
    if (secondaryController.getRightBumperButtonPressed()) {
      curControlMode = ControlMode.MANUAL;
      System.out.println("Control Mode: " + curControlMode);
      setRumbleBoth(controllers.SECONDARY, 0.1, 1);
    } else if (secondaryController.getLeftBumperButtonPressed()) {
      curControlMode = ControlMode.AUTO;
      System.out.println("Control Mode: " + curControlMode);
      setRumbleBoth(controllers.SECONDARY, 0.1, 1);
    } else if (secondaryController.getRightTriggerAxis() > 0.5 && secondaryController.getLeftTriggerAxis() > 0.5) {
      curControlMode = ControlMode.OHNO_MANUAL;
      System.out.println("Control Mode: " + curControlMode);
      setRumbleBoth(controllers.SECONDARY, 0.1, 1);
    }
  }

  private void AutoMode() {
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      updateControlMode();
      return;
    }
    if (primaryController.getAButtonPressed()) {
    } // TODO: run algae intake auto based on web dash
    if (primaryController.getXButtonPressed()) {
    } // TODO: run coral intake auto based on web dash
    if (primaryController.getBButtonPressed())
      new GoToSourceAndIntake().schedule();

  }

  private void ManualMode() {
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      updateControlMode();
      return;
    }
    if (secondaryController.getRightBumperButtonPressed())
      new SequentialCommandGroup(
          new SetHeight(Constants.ElevatorHeights.sourceIntake), new SourceCoralIntake()).schedule();
    if (secondaryController.getLeftBumperButtonPressed())
      new EndEffectorScoreCoral(0.8).schedule();
    // elevator.setHeight(elevator.getTarget() - getRightY(controllers.SECONDARY) *
    // 0.9);
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
    if (primaryController.getYButtonPressed()) {
      SwerveDrive.odometry.resetGyro();
    }
    if (secondaryController.getPOV() == 180) {
      elevator.setHeight(ElevatorHeights.minimum);
    }

    if (primaryController.getXButtonPressed()) {
      fieldOriented = !fieldOriented;
      RobotContainer.swerveDrive.setDefaultCommand(
          new SwerveDriveManualControl(
              RobotContainer.swerveDrive,
              Constants.Bot.maxChassisSpeed,
              Constants.Bot.maxChassisTurnSpeed,
              fieldOriented));
    }
  }

  private void OHNOManualMode() {
    if (secondaryController.getLeftStickButton() && secondaryController.getRightStickButton()) {
      updateControlMode();
      return;
    }

    if (primaryController.getStartButtonPressed()) {
      fieldOriented = !fieldOriented;
      RobotContainer.swerveDrive.setDefaultCommand(
          new SwerveDriveManualControl(
              RobotContainer.swerveDrive,
              Constants.Bot.maxChassisSpeed,
              Constants.Bot.maxChassisTurnSpeed,
              fieldOriented));
    }

    if (primaryController.getYButtonPressed()) {
      SwerveDrive.odometry.resetGyro();
    }

    double speed = -getRightY(controllers.SECONDARY);
    if (Elevator.elevatorEnabled) {
      elevator.LMot.set(speed);
      elevator.RMot.set(speed);
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
      new IntakeAlgaeReef(endEffector, ElevatorHeights.reefAlgaeL1Height).schedule();
    }

    if (secondaryController.getBackButtonPressed()) {
      // Get Algae L3
      new IntakeAlgaeReef(endEffector, ElevatorHeights.reefAlgaeL2Height).schedule();
    }

    if (secondaryController.getRightTriggerAxis() > Constants.Controller.triggerThreshold) {
      // Score coral
      endEffector.setManipulatorSpeed(Constants.MotorSpeeds.EndEffector.manipulatorScore);
    } else {
      endEffector.setManipulatorSpeed(0);
    }

    if (secondaryController.getLeftBumperButtonPressed()) {
      // Ground Intake
      // new GroundCoralIntake(GroundIntake.getInstance(),
      // Elevator.getInstance()).schedule();
    }

    if (secondaryController.getRightBumperButton()) {
      // Source Intake
      // new SourceCoralIntake().schedule();
      EndEffector.getInstance().rightManipulatorMotorMN.set(-secondaryController.getLeftY() * 0.8);
      EndEffector.getInstance().leftManipulatorMotorMN.set(-secondaryController.getLeftY() * 0.8);
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

  private void testingMode() {
    endEffector.algaeIntakeRotateMotorN.set(-getRightY(controllers.SECONDARY) * 0.25);
    endEffector.algaeIntakeMotorN.set(-getLeftY(controllers.SECONDARY));
  }

  public void periodic() {
    NetworkTables.driveModeManual_b.setBoolean(curControlMode == ControlMode.MANUAL);
    if (testing) {
      testingMode();
      return;
    }
    switch (curControlMode) {
      case AUTO:
        AutoMode();
        break;
      case MANUAL:
        ManualMode();
        break;
      case OHNO_MANUAL:
        OHNOManualMode();
        break;
      default:
        // Integral to the code base DO NOT CHANGE! (Copilot did it!)
        throw new IllegalStateException("Invalid control mode: \n Nuh uh, no way, not gonna happen");
    }
  }
}
