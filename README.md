# Lab4
Mapping

## How to build your project
 
### Step 1 (in terminal)

pull the repository to your pc or jupytherhub environment

    git clone git@github.com:cu-csci-3302-IntroductionToRobotics/Lab1.git

### Step 2 (in your linux environment)

Open webots application and then from menu bar go to `File->OpenWorld...` and choose the webot world file in `lab*/world` directory of this repository

at this point you should be able to see the robot in a 3D environment and be able to rotate the camera while left clicking in the scene and moving your mouse.

### Step 3 (in vscode)

- go to menu bar(might be a three stacked horizontal lines icon on top left corner of your vscode env) and then click `File->Open Folder...` and choose the lab folder that you pulled on Step0

- open up a terminal from the menu bar `Terminal->NewTerminal`

### Step 4 (in vscdoe terminal)

**Skip this step if you are on jupytherhub env**

first check if WEBOTS_HOME Environment variable is defined.
use the following command on terminal

    echo $WEBOTS_HOME

response to this should show the path for where webots is installed. if you get a blank response then define it with

    export WEBOTS_HOME=\path\to\your\webots\install\directory

### Step 5 (in vscode terminal)

  go to build folder

    cd build

then run cmake (this will find requred libraries for your executable)

    cmake ..

then build your executables

    make

at this point you should see your run_lab executable in the build folder

    ls

## How to run your simulation

have both your webots simulator environmnet and vscode open side by side in your window (or on two monitors if you have them on your pc setup).

press the play button on your webots simulator window (this starts the simulation but since your code is not running nothing will move)

run your executable from thermianl

    ./run_lab

at this point if your code is effecting the robot then you'll see it moving in the simulator

## How to use Dubugger

first put some debug points on your code line (where ever you want to pause and debug your code).

Go to debug menu of vscode (little play/bug icon on the left bar) and press the green play button.

this would run your code up to the breakpoint and pause on the breakpoint and you can inspect your variable values as needed. (this action would also pause the simulator while code line is waiting on the breakpoint)

## Folder setup
