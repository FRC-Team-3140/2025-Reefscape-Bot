package frc.robot.libs;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class LoggedCommand extends Command {
    int index = 0;

    public void initialize() {
        CommandLogger.commandNames.add(this.getName());
        CommandLogger.commandStates.add(1);
        // TODO: Fix issue where old commands aren't cleared when number of logged commands is decreased. 
        if (CommandLogger.commandNames.size() > NetworkTables.numOLoggedCmds_i.getInteger(50)) {
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