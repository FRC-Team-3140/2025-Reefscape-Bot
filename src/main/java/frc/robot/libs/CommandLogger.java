package frc.robot.libs;

import java.util.ArrayList;

public class CommandLogger {
    static ArrayList<String> commandNames = new ArrayList<String>();
    static ArrayList<Integer> commandStates = new ArrayList<Integer>();

    public CommandLogger() {
        // Constructor
    }

    public static void updateNetworktables() {
        String[] commandNamesArray = new String[commandNames.size()];
        commandNames.toArray(commandNamesArray);
        NetworkTables.commands.setStringArray(commandNamesArray);

        Integer[] commandStatesArray = new Integer[commandStates.size()];
        commandStates.toArray(commandStatesArray);
        NetworkTables.commandStatuses.setNumberArray(commandStatesArray);

    }
}