# Welcome to BobcatLib 2.0!

#### This repo contains several utilities and code templates, allowing for quick creation of a robot's codbase at the beginning of a season. This readme won't necessarily cover every single file in BobcatLib, but it will cover all the important ones 

## Overview

### PoseEstimation:

#### BobcatSwerveEstimator
This is the class you should instantiate when creating a swerve pose estimator, it has all of the functionality of WPILib's swerve estimator, but also supports dynamic odometry std devs, as well as the ability to log pure odometry values.

The pure odometry should only be used for debugging and tuning Std Devs

#### BobcatEstimator & BaseBobcatSwerveEstimator
These are the classes that do all the heavy lifting of pose estimation, do NOT instantiate these classes. BobcatSwerveEstimator essentially just contains two base swerve estimators, one of them recieves all updates and functions like normal, but the second one only recieves odometry updates, and is only used for tracking pure odometry values.

### Swerve:
I'm not going to go into too much detail, but this folder contains various classes necessary for creating a swerve drive.

#### SwerveBase
This is a base swerve drivetrain, with a bunch of general swerve features you should inherit this in your season specific swerve subsystem. Feautres include:
* Fully configured auto builder with rotation override capability
* Easy pathfinding command factories
* Pose estimation with dynamic odometry and vision trust levels
* Support for any number of Limelights
* Snap-to-angle autoalign
* Translational aim assist

#### TeleopSwerve
This is the command for controlling the robot, it should be applied as the default command for the drivetrain in teleop

#### SwerveConstants
Make sure to spend time tuning all gains in this class, to ensure maximum performance.

## Sysid:

### Sysid.java
This class abstracts out a lot of the sysid logic, for quicker tuning. To use it, create an instance in robot container, and pass in your swerve drive in the constructor. Use the getSysidTest() method to get the commands necessary for running the tests, data will be stored on the rio, and has to be retrived via the sysid tool.

### SysidCompatibleSwerve
Your swerve subsystem MUST implement this interface. It contains all of the methods that Sysid needs and expects to control the robot and log data properly.

## Team6328:
Thanks Jonah ;)

This folder contains a bunch of code that we ~~stole~~ borrowed from 6328. Some of this is code that they stole from 254.

### LoggedTunableNumber
DOES NOT WORK RN <p>
This is a double that is automatically published to NT and recorded by AK, very useful for tuning PIDs without having to redeploy every time, just make sure to get rid of it after you're done tuning.