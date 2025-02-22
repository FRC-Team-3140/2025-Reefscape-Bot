package frc.robot.libs;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class LoggedCommand extends Command {
    int index = 0;

    public void initialize() {
        CommandLogger.commandNames.add(this.getName());
        CommandLogger.commandStates.add(1);
        if (CommandLogger.commandNames.size() > 50) {
            CommandLogger.commandNames.remove(0);
            CommandLogger.commandStates.remove(0);
        }
        index = CommandLogger.commandNames.size() - 1;
        CommandLogger.updateNetworktables();
    }

    public void end(boolean interrupted) {
        CommandLogger.commandStates.set(index, interrupted ? 2 : 0);
        CommandLogger.updateNetworktables();
    }
}