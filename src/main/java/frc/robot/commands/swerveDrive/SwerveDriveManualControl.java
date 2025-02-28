package frc.robot.commands.swerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.libs.LoggedCommand;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.SwerveDrive;
//Works Well

/**
 * This class represents a basic swerve control command.
 * It is intended to be the default command for the drive.
 */
public class SwerveDriveManualControl extends LoggedCommand {
    private final Controller controller = Controller.getInstance();
    private final SwerveDrive swerveDrive; // The swerve drive subsystem
    private final double maxSpeed; // The maximum speed for the swerve drive
    private final double maxChassisTurnSpeed; // The maximum turn speed for the chassis
    private final boolean fieldRelative = false;
    private final double movementThreshold = 0.75;
    private boolean locked = false;

    /**
     * Creates a new BasicSwerveControlL2 command.
     *
     * @param swerveDrive         The swerve drive subsystem
     * @param maxSpeed            The maximum speed for the swerve drive
     * @param maxChassisTurnSpeed The maximum turn speed for the chassis
     */
    public SwerveDriveManualControl(SwerveDrive swerveDrive, double maxSpeed, double maxChassisTurnSpeed) {
        this.swerveDrive = swerveDrive;
        this.maxSpeed = maxSpeed;
        this.maxChassisTurnSpeed = maxChassisTurnSpeed;

        addRequirements(swerveDrive); // This command requires the swerve drive subsystem
    }

    /**
     * The command execution logic.
     * Gets the joystick inputs and drives the swerve drive accordingly.
     */
    @Override
    public void execute() {
        if ((controller.primaryController.getLeftStickButtonPressed()
                && controller.primaryController.getRightStickButtonPressed())
                || (locked && ((Math.abs(controller.primaryController.getLeftX()) > movementThreshold)
                        || (Math.abs(controller.primaryController.getLeftY()) > movementThreshold)
                        || (Math.abs(controller.primaryController.getRightX()) > movementThreshold)))) {
            locked = !locked;
        }

        if (!locked) {
            // Calculate the x speed based on the joystick input
            final double xSpeed = -controller.getLeftY(Controller.controllers.PRIMARY) * maxSpeed;

            // Calculate the y speed based on the joystick input
            final double ySpeed = -controller.getLeftX(Controller.controllers.PRIMARY) * maxSpeed;

            // Calculate the rotation speed based on the joystick input
            final double rot = -controller.getRightX(Controller.controllers.PRIMARY) * maxChassisTurnSpeed;

            if (Math.abs(xSpeed) > 0.0 || Math.abs(ySpeed) > 0 || Math.abs(rot) > 0) {
                swerveDrive.drive(xSpeed, ySpeed, rot, fieldRelative); // Drive the swerve drive
            } else {
                double[] curModuleAngles = new double[swerveDrive.modules.length];

                for (int i = 0; i < swerveDrive.modules.length; i++) {
                    curModuleAngles[i] = swerveDrive.modules[i].getState().angle.getDegrees();
                }

                SwerveModuleState[] states = new SwerveModuleState[swerveDrive.modules.length];

                for (int i = 0; i < swerveDrive.modules.length; i++) {
                    // This will ensure that the bot doesn't move while maintaining a certain angle
                    // Currently it will hold the angle it's currently at to prevent the wheels from
                    // snapping back and forth between 0 and a controller setpoint.
                    states[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(curModuleAngles[i]));
                }

                swerveDrive.setSwerveModuleStates(states, false);
            }
        } else {
            swerveDrive.setSwerveModuleStates(Constants.Bot.defaultSwerveStates, true);
        }
    }

    /**
     * Determines whether the command is finished.
     * If this command is the default command for the drive, it should never finish.
     *
     * @return false because this command should never finish if it's the default
     *         command for the drive
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}