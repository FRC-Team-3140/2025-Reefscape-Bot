# Can IDs
| Motor Name       | CAN ID |
|------------------|--------|
| FLNeo            | 0      |
| FLVortex         | 1      |
| FRNeo            | 2      |
| FRVortex         | 3      |
| BLNeo            | 4      |
| BLVortex         | 5      |
| BRNeo            | 6      |
| BRVortex         | 7      |
| ElevLNeo         | 8      |
| ElevRNeo         | 9      |
| EELeft           | 10     |
| EERight          | 11     |
| EETop            | 12     |
| AIRotate         | 13     |
| AIIntake         | 14     |
| GILeft           | 15     |
| GIRight          | 16     |
| GILift           | 17     |

# Running Localy For Development

## Robot Code:
1) Open the robot code in wpilib vscode. 
2) Ctrl + Shift + P, type Simulate, then select "WPILib: Simulate Robot code". 
Glass will then open and allow you to set the bot state, use the controller, and look through networktables. Currently there aren't any physics sims, so the bot and subsystem encoders aren't updated.

## Web Dashboard (will show disconnected unless a bot or bot sim is running):
1) Open the dashboard code in an IDE. 
2) Change the address on the NT4Provider element in src/main.tsx to localhost. (If using a physical bot, do not change it)
3) Set up the environment to use Node.js. The app was developed for and on the pi, so Linux is the only tested platform. Remember to run `npm i` when setting up the project to install dependencies.
4) Run `npm run dev`.
The webserver urls will be printed in the terminal and can be accessed from a web browser.