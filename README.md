# ECE 470 Final Project
This is our repository for the ECE 470 final project.  Our goal is to run a simulation of the robot Baxter lifting weights.  It will try different lifts and have good form.

# Dependencies
You need python 3.6, numpy, scipy, sympy, and matplotlib.  And of course V-Rep needs to be installed.

# How to Run (Tested on Windows)
Clone this repository into your V-Rep folder.  A sample path would be "C:\Program Files\Vrep\ece470-project."  Copy the scene file ("ArnoldV2_hchung13_sidneyo2.ttt") into the scenes folder in this directory: "C:\Program Files\Vrep\V-REP_PRO_EDU\scenes."  The folder "Vrep" had a different name when it was first installed (we changed it to make it easier to type), so be sure to follow whatever paths you have in your V-Rep installation.  

Once the setup is complete, open up the V-Rep simulator and open the scene that you copied into the scenes folder.  Then open command prompt in the "C:\Program Files\Vrep\ece470-project" directory and run the command "python demo.py."

# For OS Other Than Windows
The above How-to may not cover the exact steps you need to take if your operating system is not Windows.  An alternative would be to just take our "demo.py" file and put it in your working V-Rep folder, making sure that your folder also has the correct dependent files that work on your OS.  You'll also still need to put our .ttt scene file in your scenes folder to have access to it in V-Rep.
