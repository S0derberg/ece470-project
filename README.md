# ECE 470 Final Project
This is our repository for the ECE 470 final project.  Our goal is to run a simulation of the robot Baxter lifting weights.  It will try different lifts and have good form.

# Dependencies
You need python 3.6, numpy, scipy, sympy, and matplotlib.  And of course V-Rep needs to be installed.

# Getting V-REP
We are using the V-Rep simulator on Windows.  Install the simulator from http://www.coppeliarobotics.com/previousversions.html.  We got the Windows 64 V-REP PRO EDU version 3.4.0.  A sample path to the installation would be"C:\Program Files\Vrep." The folder "Vrep" had a different name when it was first installed (we changed it to make it easier to type), so be sure to follow whatever paths you have in your V-Rep installation.

# How to Run (Tested on Windows)
Clone this repository into your V-Rep folder.  A sample resulting path would be "C:\Program Files\Vrep\ece470-project."  Copy the scene file ("ArnoldV2_hchung13_sidneyo2.ttt") into the scenes folder in this directory: "C:\Program Files\Vrep\V-REP_PRO_EDU\scenes." The following parts represent the weekly deadlines.  Part 1 was the first deadline, part 2 is second, etc.  We will add new parts each week while keeping the rest of the README the same.

## Part 1
Once the setup is complete, open up the V-Rep simulator and open the scene that you copied into the scenes folder.  Then open command prompt in the "C:\Program Files\Vrep\ece470-project" directory and run the command "python demo.py."  This script will start a simulation and test all the joint limits of the left and right hands, rotate the torso, and close and open the hands.

## Part 2
To run the next part, run the command "python forward.py."  This script will start a simulation, move the dummy reference frames to predicted poses from our forward kinematics calculation, and then move the arms to a set of joint variables.  This repeats 3 times using different sets of joint variables.  The pose of the Jaco hand should match up with the dummy reference frames.  Our additional work this week was to get the two arms moving at the same time, and to be mirror images of each other.  This will be essential for doing lifts that require both arms.

## Part 3
This part is to simulate inverse kinematics.  It can be run with the command "python inverse.py."  This code will ask for user input for the goal frame by asking seven questions.  It asks for which arm to move (left or right), the x, y, and z translation of the goal pose from the origin, and the sequential rotations of the goal pose around its own x, y, and z axes.  A couple of suggested poses are listed below.  Once the pose is inputted, the code uses numerical inverse kinematics to find a set of joint angles to achieve the goal pose.  If the code can't converge on a set or if it repeatedly finds invalid sets (joints outside of joint limits), then Baxter will indicate this inability by rotating his torso 180 degrees clockwise and counterclockwise.  If the code does converge on a valid joint angle set, Baxter will move to reach the goal pose.  This process of prompting for user input and trying to reach the goal pose repeats 4 times and then the simulation ends.  

Our additional work for this week was to add the use of the torso rotation joint.  We did this by adding another screw axis to the beginning of both the left and right screw matrices.  Then the inverse kinematics process takes the ability to rotate the torso into account.  Now our robot can reach more goal poses behind it and prefers to reach these by rotating its torso first and then minimally rotating the arm joints rather than performing some complicated arm rotation.

### How To Dab
Enter these values for the user input:<br>
Arm: right<br>
X-Translation: 1.1<br>
Y-Translation: 0.3<br>
Z-Translation: 1.3<br>
X-Rotation: -90<br>
Y-Rotation: 45<br>
Z-Rotation: 0<br>
<br>
Then these:<br>
Arm: right<br>
X-Translation: 0.5<br>
Y-Translation: 0<br>
Z-Translation: 1.8<br>
X-Rotation: -90<br>
Y-Rotation: 0<br>
Z-Rotation: 0<br>
<br>
After that do these:<br>
Arm: left<br>
X-Translation: 0<br>
Y-Translation: 1.2<br>
Z-Translation: 1.9<br>
X-Rotation: -60<br>
Y-Rotation: 0<br>
Z-Rotation: 0<br>

## Part 4
This part is collision detection.  The command to run this time is "python collision.py".  This will show the robot going into three different base configurations.  At each of these, the robot will move its arms and torso through some path.  The first base configuration has 10 total sub-configurations on the path, and the next two have 14.  Also, to notify of collisions, we place dummy spheres in the scene on the ground.  The sphere will be green if there was no collision and red if there was a collision.  Expect to see collision during the paths starting at the second two configurations (first colliding with the rack, then with itself).  Our extra work this week was to start figuring out the motions for the weightlifting.  The first path resembles a curl, as does the third one (albeit that one curls wrong and collides).

## Part 5
This part is motion planning.  The command to run it is "python planning.py".  This code has three goal poses that it tries to reach.  For each one, the code uses inverse kinematics to find the goal thetas that would reach the pose.  Then it uses the sampling method from class to find thetas in free space that can either be added to the start tree or the goal tree.  Once a theta is found that gets added to both trees, the final path is constructed and Baxter moves along the path.  The path is a series of straight line segments between thetas.  The paths are usually around 3 or 4 thetas (not including start and goal) and Baxter pauses briefly between each segment.  If Baxter does not find a path that avoids collision fast enough, it'll just spin around to indicate failure.

We changed our scene a bit to reduce the number of spheres to check with.  Now Baxter tries to avoid hitting Bill, who is wisely standing right in front of a large robot.  We chose goal poses that involve one arm moving across Baxter toward the other arm.  These kinds of motions would usually just go straight to the goal pose, but now Bill is in the way.  Instead, expect to see longer paths in the video that involve Baxter lifting his arm and going over Bill's head and then down to the goal pose.  This demonstrates Baxter's ability to avoid obstacles while also not colliding with itself. 

For our extra work, we both improved upon our inefficient motion planning solutions in our homework (that were either using too much space or being computationally inefficient).  We now use a Node structure that keeps track of a set of thetas and a parent Node.  This allows for easy path reconstruction once the final set of thetas is found.

## Part 6 (Final Part)
For the last assignment, we combine the work from the previous parts to achieve our goal of having Baxter lift a weight.  Be sure to get the newest V-Rep scene file.  The command to run it is "python lift.py". This will recreate the activity in our final project video.  It starts by automatically placing dummies in the table on which the dumbbells are resting.  These are placed so that Baxter does not collide with the table.  Next, Baxter will plan a path to a spot above and behind the bigger dumbbell.  After moving, he will make two more movements to get close to the dumbbell.  Then Baxter will grip the dumbbell, lift it up, rotate to not face the table, lower the weight, and then lift it in the form of a curl.  Lastly, Baxter dabs and then drops the weight.

# For OS Other Than Windows
The above How-to may not cover the exact steps you need to take if your operating system is not Windows.  An alternative would be to just take our python files and put it in your working V-Rep folder, making sure that your folder also has the correct dependent files that work on your OS.  You'll also still need to put our .ttt scene file in your scenes folder to have access to it in V-Rep.
