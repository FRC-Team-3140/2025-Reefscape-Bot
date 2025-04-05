// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.CycleHorizontal;
import frc.robot.commands.Autos.CycleVertical;
import frc.robot.commands.compoundCommands.SourceCoralIntake;
import frc.robot.commands.elevator.ReturnToStowed;
import frc.robot.commands.elevator.SetHeight;
import frc.robot.commands.endeffector.EndEffectorScoreCoral;
import frc.robot.commands.swerveDrive.Align;
import frc.robot.commands.swerveDrive.Drive;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.*;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.tests.TestReefPoses;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static RobotContainer container = null;

  // The robot's subsystems and commands are defined here...
  public static SwerveDrive swerveDrive = SwerveDrive.getInstance();
  public static Elevator elevator = Elevator.getInstance();
  public static EndEffector endEffector = EndEffector.getInstance();
  public static TestRunner testRunner = TestRunner.getInstance();
  public static Controller controller = Controller.getInstance();
  public static Odometry odometry = Odometry.getInstance();
  public static Camera camera = Camera.getInstance();
  public static SignalTower leds = SignalTower.getInstance();

  private SendableChooser<String> reefSide = new SendableChooser<>();
  private SendableChooser<String> cycleDirection = new SendableChooser<>();
  private SendableChooser<Integer> reefLevel = new SendableChooser<>();
  private SendableChooser<Command> PathPlanner = new SendableChooser<>();

  // WARNING: These booleans will override auto selectiosn!
  private boolean testReefPoses = false;
  private boolean pushAutoMode = false;

  // Get the singleton instance or create it if it doesn't exist
  public static RobotContainer getInstance() {
    if (container == null) {
      container = new RobotContainer();
    }
    return container;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    // Add Named Commands for Pathplanner override autos
    NamedCommands.registerCommand("Return To Stowed", new ReturnToStowed());
    NamedCommands.registerCommand("Align Score L4",
        new SequentialCommandGroup(
            new ParallelCommandGroup(new Align(), new SetHeight(Constants.ElevatorHeights.reefCoralL4Height)),
            new EndEffectorScoreCoral(0.8)));
    NamedCommands.registerCommand("Intake", new SourceCoralIntake());

    // Configure the auto options
    reefSide.addOption("Closest", "Closest");
    reefSide.addOption("Back", "Back");
    reefSide.addOption("Back Left", "Back Left");
    reefSide.addOption("Front Left", "Front Left");
    reefSide.addOption("Front", "Front");
    reefSide.addOption("Front Right", "Front Right");
    reefSide.addOption("Back Right", "Back Right");

    reefLevel.addOption("L4", 4);
    reefLevel.addOption("L3", 3);
    reefLevel.addOption("L2", 2);
    reefLevel.addOption("L1", 1);

    cycleDirection.addOption("Horizontal", "Horizontal");
    cycleDirection.addOption("Vertical", "Vertical");

    PathPlanner.setDefaultOption("Normal - No PathPlanner", null);
    PathPlanner.addOption("Mobility", AutoBuilder.buildAuto("Simple Mobility"));
    PathPlanner.addOption("Far Simple", AutoBuilder.buildAuto("Far Simple"));
    PathPlanner.addOption("1 Coral", AutoBuilder.buildAuto("1 Coral Override"));
    PathPlanner.addOption("2 Coral L", AutoBuilder.buildAuto("2L Coral Override"));
    PathPlanner.addOption("2 Coral R", AutoBuilder.buildAuto("2R Coral Override"));

    SmartDashboard.putData("Starting Side", reefSide);
    SmartDashboard.putData("Reef Level", reefLevel);
    SmartDashboard.putData("Cycle Direction", cycleDirection);
    SmartDashboard.putData("PathPlanner", PathPlanner);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (pushAutoMode)
      return new Drive(10000000, false, Constants.Bot.maxChassisSpeed / 2, 0, 0);

    if (testReefPoses)
      return new TestReefPoses();

    String cycleDirectionSelected = cycleDirection.getSelected();
    String reefSideSelected = reefSide.getSelected();
    Integer reefLevelSelected = reefLevel.getSelected();

    ////////// Defaults //////////
    if (cycleDirectionSelected == null) {
      if (PathPlanner.getSelected() != null) {
        System.out.println("Pathplanner");
        return PathPlanner.getSelected();
      }

      cycleDirectionSelected = "Horizontal";
    }

    if (reefSideSelected == null) {
      if (PathPlanner.getSelected() != null) {
        System.out.println("Pathplanner");
        return PathPlanner.getSelected();
      }

      reefSideSelected = "Closest";
    }

    if (reefLevelSelected == null) {
      if (PathPlanner.getSelected() != null) {
        System.out.println("Pathplanner");
        return PathPlanner.getSelected();
      }

      reefLevelSelected = 4;
    }

    //////////////////////////////

    if (cycleDirectionSelected.equals("Horizontal")) {

      int level = reefLevelSelected != null ? reefLevelSelected : 0;

      switch (reefSideSelected) {
        case "Closest":
          return new CycleHorizontal(level, elevator, odometry);

        case "Back":
          return new CycleHorizontal(level, 0, elevator, odometry);

        case "Back Left":
          return new CycleHorizontal(level, 1, elevator, odometry);

        case "Front Left":
          return new CycleHorizontal(level, 2, elevator, odometry);

        case "Front":
          return new CycleHorizontal(level, 3, elevator, odometry);

        case "Front Right":
          return new CycleHorizontal(level, 4, elevator, odometry);

        case "Back Right":
          return new CycleHorizontal(level, 5, elevator, odometry);

        default:
          System.err.println("Somehow magically selected something impossible...");
          return null;
      }
    } else if (cycleDirectionSelected.equals("Vertical")) {
      switch (reefSideSelected) {
        case "Closest":
          return new CycleVertical(elevator, odometry);

        case "Back":
          return new CycleVertical(0, elevator, odometry);

        case "Back Left":
          return new CycleVertical(1, elevator, odometry);

        case "Front Left":
          return new CycleVertical(2, elevator, odometry);

        case "Front":
          return new CycleVertical(3, elevator, odometry);

        case "Front Right":
          return new CycleVertical(4, elevator, odometry);

        case "Back Right":
          return new CycleVertical(5, elevator, odometry);

        default:
          System.err.println("Somehow magically selected something impossible...");
          return null;
      }
    } else {
      System.err.println("Somehow magically selected something impossible...");
      return null;
    }
  }
}