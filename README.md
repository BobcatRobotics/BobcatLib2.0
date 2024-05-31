# Welcome to BobcatLib 2.0!

#### This repo contains several utilities and code templates, allowing for quick creation of a robot's codbase at the beginning of a season. This readme won't necessarily cover every single file in BobcatLib, but it will cover all the important ones 


## Annotations:
This class contains some basic markers to help determine how a class in BobcatLib should be used.
Classes marked with @SeasonBase should NOT be modified to fit your season-specific needs, but should be instantiated, inherited or are static. @SeasonSpecific classes SHOULD be modified, these classes are either constants that need to be updated, or are subsystem templates that you can copy-paste into your project's Subsystems folder. This does NOT mean that you shouldn't make changes to @SeasonBase classes throughout the season! if you have improvements, don't hesitate to add them, however, whatever you add should be helpful every season. 


## Candle:

This folder contains a basic templated CANdle subsystem with AdvantageKit, copy it over to your seasons subsystem folder

## Gamepads

#### EightBitDo
This is a wrapper around a CommandJoystick, containing triggers representing each button, allowing for users to access buttons without having to memorize their respective indicies. 


## PoseEstimation:

#### BobcatSwerveEstimator
This is the class you should instantiate when creating a swerve pose estimator, it has all of the functionality of WPILib's swerve estimator, but also supports dynamic odometry std devs, as well as the ability to log pure odometry values, so you can see what your pose would look like with no vision measurements.

#### BobcatEstimator & BaseBobcatSwerveEstimator
These are the classes that do all the heavy lifting of pose estimation, do NOT instantiate these classes. They are essentially the same as WPILib's SwerveDrivePoseEstimator and PoseEstimator classes, but modified to support dynamic state stdDevs.

BobcatSwerveEstimator essentially just contains two base swerve estimators, one of them recieves all updates and functions like normal, but the second one only recieves odometry updates, and is only used for tracking pure odometry values.

## Swerve:

This folder contains various classes necessary for creating a swerve drive.

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

NOTE: Drivetrains passed into this class must implement the SysidCompatibleSwerve interface, BaseSwerve already implements this.

## Vision:

#### Vision


