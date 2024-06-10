
## Annotations: 
This class contains markers to help determine how a class in BobcatLib should be used.
Classes marked with `@SeasonBase` should NOT be modified to fit your season-specific needs, but should be instantiated, inherited or are static. `@SeasonSpecific` classes SHOULD be modified, these classes are either constants that need to be updated, or are subsystem templates that you can copy-paste into your project's Subsystems folder. This does NOT mean that you shouldn't make changes to `@SeasonBase` classes throughout the season! if you have improvements, don't hesitate to add them, however, whatever you add should be helpful every season. 


## Candle:

This folder contains a basic templated CANdle subsystem with AdvantageKit. You can configure all the states you want in `CANdleState`. You can set an animation for a specific amount of time, or you can set it to persist until the next animation is sent to it.

## Gamepads

This is a wrapper around a CommandJoystick, containing triggers representing each button, allowing for users to access buttons without having to memorize their respective indicies. Each class in here represents a different controller. 


## PoseEstimation:

#### BobcatSwerveEstimator
This is the class you should instantiate when creating a swerve pose estimator, it has all of the functionality of WPILib's swerve estimator, but also supports dynamic odometry std devs, as well as the ability to log pure odometry values, so you can see what your pose would look like without vision measurements.

#### BobcatEstimator & BaseBobcatSwerveEstimator
These are the classes that do all the heavy lifting of pose estimation, do NOT instantiate these classes. They are essentially the same as WPILib's `SwerveDrivePoseEstimator` and `PoseEstimator` classes, but modified to support dynamic state(odometry) stdDevs.

`BobcatSwerveEstimator` essentially just contains two `BaseBobcatSwerveEstimators`, one of them recieves all updates and functions like normal, but the second one only recieves odometry updates, and is only used for tracking pure odometry values.

## Swerve:

This folder contains various classes necessary for creating a swerve drive.

#### SwerveBase
This is a base swerve drivetrain, with a bunch of general swerve features you should inherit this in your season specific swerve subsystem. Feautres include:
* Fully configured PathPlanner auto builder with rotation override capability
* Easy pathfinding command factories
* Pose estimation with dynamic odometry and vision trust levels
* Support for any number of Limelights
* Snap-to-angle autoalign
* Translational aim assist
* Easy sysid interoperability

#### TeleopSwerve
This is the command for controlling the robot, it should be applied as the default command for the drivetrain in teleop

#### SwerveConstants
Swerve related constants. See the [quickstart](QUICKSTART.md) for more info on how to set this up.

## Sysid:

### Sysid.java
System identification is a process of running different tests on your drivetrain to figure out how it responds to different voltages, see the [wpilib docs](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html) for more info. Basically, there three gains you need to determine, kS, kV, and kA. kS is the constant static force experienced by your drivetrain, kV is the voltage needed to maintain a certain velocity, and kA is the voltage needed to accelerate or decelerate between velocities. This class abstracts out a lot of the sysid logic for quicker tuning. See the [quickstart](QUICKSTART.md) for more info on how to use it.

NOTE: Drivetrains passed into this class must implement the SysidCompatibleSwerve interface, BaseSwerve already implements this.

## Vision:

#### Vision
