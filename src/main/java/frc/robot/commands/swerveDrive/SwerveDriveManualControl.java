package frc.robot.commands.swerveDrive;

import frc.robot.Constants;
import frc.robot.libs.LoggedCommand;
import frc.robot.libs.NetworkTables;
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
    private final double movementThreshold = 0.75;
    private boolean fieldRelative = true;
    private boolean locked = false;

    /**
     * Creates a new BasicSwerveControlL2 command.
     *
     * @param swerveDrive         The swerve drive subsystem
     * @param maxSpeed            The maximum speed for the swerve drive
     * @param maxChassisTurnSpeed The maximum turn speed for the chassis
     */
    public SwerveDriveManualControl(SwerveDrive swerveDrive, double maxSpeed, double maxChassisTurnSpeed,
            boolean fieldRelative) {
        this.swerveDrive = swerveDrive;
        this.maxSpeed = maxSpeed;
        this.maxChassisTurnSpeed = maxChassisTurnSpeed;
        this.fieldRelative = fieldRelative;

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

        if (controller.primaryController.getBackButtonPressed())
            fieldRelative = !fieldRelative;

        if (!locked) {
            // Calculate the x speed based on the joystick input
            final double xSpeed = -controller.getLeftY(Controller.controllers.PRIMARY) * maxSpeed;

            // Calculate the y speed based on the joystick input
            final double ySpeed = -controller.getLeftX(Controller.controllers.PRIMARY) * maxSpeed;

            // Calculate the rotation speed based on the joystick input
            final double rot = -controller.getRightX(Controller.controllers.PRIMARY) * maxChassisTurnSpeed;

            swerveDrive.drive(xSpeed, ySpeed, rot, fieldRelative); // Drive the swerve drive
        } else {
            swerveDrive.setSwerveModuleStates(Constants.Bot.defaultSwerveStates, true);
        }

        NetworkTables.fieldOriented_b.setBoolean(fieldRelative);
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