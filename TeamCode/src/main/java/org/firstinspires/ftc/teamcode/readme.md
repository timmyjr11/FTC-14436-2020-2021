This is all of the code I have ever written for the 2020-2021 FIRST FTC season Ultimate Goal. Note that there is quite literally little to no commenting on these codes because
of the fact I did not have to teach anyone on how to code during this time, from this point forward we WILL be commenting on all codes for the up coming 2021-2022 season.

In order to access the codes, you have to go into the files in this specific order: </br>
FtcRobotController-master/FtcRobotController-master > TeamCode > src/main > org/firstinspires/ftc/teamcode

'roadRunnerStuff' contains anything and everything RoadRunner, which means it also includes the final versions for the tele-op and auto:</br> <br>
Final Tele-op == 'HalcyonState.java' </br>
Final Auto == 'notSoScuffedAuto.java' </br> </br>
Every other files listed in RoadRunnerStuff are either config files that allowed roadrunner to work ('DriveConstants.java', 'PoseStorage.java', etc.), or test files/older versions
of code that were used to build upon and create the final codes that were used in state.

'Shooter' were for the motors that shot the rings, more specifically the PID aspect to them. PID is outside the range of what this file is for, so if you want more information
about PID and why it's important, talk to me about it and review the codes within 'Shooter' to understand the logic behind PID. (Note: This does not exist anymore due to problems with a library called "Jotai"
so this will not be here. Nevertheless, it is important to learn PID, so please do research on that) </br>

'preRoadRunner' is the original folder I first programmed before I learned RoadRunner. This is the folder you get when you first install the FTC SDK for the 2020-2021
season. All of the codes included in the folder contains code that was programmed without RoadRunner, so the newest code created here was around February 2021, which was the first or
second competition we participated in. Notable codes in this folder is 'Hbot.java' and 'scuffedAuto.java' because they were the final versions of code that did not use RoadRunner. </br>

'testAndCopies' folder contains anything and everything that involved doing a test with a certain code and old versions of code from Halcyon (Hbot).
there are even codes that were archived that dated back to December, when we first used started using text based coding.