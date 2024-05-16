# Standards and Expectations

## Table of Contents

I. [General Standards](#general-standards)

  1. [Formatting](#formatting)
  2. [Commit Messages](#commit-messages)
  3. [Branching](#branching)
  4. [Pull Requests](#pull-requests)
  5. [Documentation](#documentation)
  6. [Testing](#testing)
  7. [Issue Tracking](#issue-tracking)
  8. [Prioritization](#prioritization)
  9. [Open Alliance Posts](#open-alliance-posts)
  10. [Competitions](#competitions)

II. [Specifics](#specifics)

  1. [Robot Container](#robot-container)
  2. [Swerve](#swerve)
  3. [Scoring Mechanism](#scoring-mechanism)
  4. [Endgame Mechanisms](#endgame-mechanisms)
  5. [Teleop](#teleop)
  6. [Autonomous](#autonomous)
  7. [Vision](#vision)
  8. [PID Loops](#pid-loops)
  9. [Logging](#logging)
  10. [Simulation](#simulation)

## General Standards

### Formatting

#### File Naming

- File names should be written in PascalCase and should be descriptive of the file's contents.
- Subsystem files should follow the AdvantageKit naming convention. For example:
  - `Arm.java`
  - `ArmIO.java`
  - `ArmIOKraken.java`
  - `ArmIOSim.java`

#### File Structure

- All files should be places in the relevent folders. For example:
  - Subsystems should be placed in the `subsystems` folder.
  - Commands should be placed in the `commands` folder.
  - Libraries and classes borrowed from other teams should be placed in the `lib` folder.
  - Subsystem-specific files should be placed in the relevant sub-folders.
- `Constants.java` should hold all constant values and should be accessed from all other classes.

#### Code Style

- Variables:
  - Variables should be written in camelCase.
  - Names should be descriptive and include relevant units.
  - Variable should use `final` or `static` when appropriate.
  - Variables should be declared at the smallest scope possible.
  - Constants should be declared in the `Constants.java` file and should be written in all caps with underscores separating words.
- Methods:
  - Methods should be written in camelCase.
  - Names should be descriptive of the method's purpose.
  - All methods should include a Javadoc comment (see [Documentation](#documentation)).
  - Methods should be declared `public` or `private` when appropriate.
  - Methods should be declared `static` when appropriate.
  - Methods should use the `@Override` annotation when overriding a method from a superclass.
  - Methods should make use of the `@AutoLogOutput` annotation when convenient.
- Classes:
  - Classes should be written in PascalCase.
  - Names should be descriptive of the class's purpose.
- Syntax:
  - All code should be indented properly.
  - Try to avoid complicated one-liners.
  - Conditions in if statements should be surrounded by parentheses and should be separated onto multiple lines if necessary.

### Commit Messages

- Commit messages should begin with the initials of the person(s) who made the commit. Example: "[DH] Commit message ..."
- Commit messages should be descriptive of ALL changes made in the commit.
- Commits should be made frequently and should be pushed to the repository at the end of each meeting or day.

### Branching

- Branches should be created for each new feature or bug fix.
- Branch names should be descriptive of the feature or bug being worked on. Avoid putting emojis in branch names.
- Branches should be deleted after they have been merged into the main branch.

### Pull Requests

- Pull requests should be created for each branch that is ready to be merged into the main branch.
- Pull requests should be reviewed by at least one mentor and one student lead before being merged.
- Pull requests should be descriptive of the changes made in the branch.
- Pull requests should be closed after the branch has been merged.
- Pull requests should be linked to the issue(s) that they are addressing.
- Branches should pull from main before creating a pull request.
- Branches to be pulled need to be tested and working before being merged.

### Documentation

- All code should be documented.
- Javadoc comments should be used for all methods ans should include a description of the method's purpose, parameters, and return value, including units where relevant.
- Every class should include a comment at the top descibing the purpose of the class and any important information about it. This comment should be descriptive enough that someone who has never seen the class before can understand what it does.
- All constants should be documented in the `Constants.java` file and should have self-documenting names.

### Testing

- All code should be tested before being merged into the main branch.
- Results from the testing should be added to the code's documentation when appropriate.
- Some changes can be tested using simulation before being tested on the robot. This is encouraged when possible.

### Issue Tracking

- All issues should be tracked using GitHub's issue tracking system.
- Issues should be descriptive of the problem or feature being addressed.
- Issues should be linked to the pull requests that address them.
- Whoever is working on the issue should assign themselves to it on GitHub.

### Prioritization

- Issues that are critical to the robot's functionality should be prioritized over other issues.
- Issues that are blocking other issues should be prioritized over other issues.
- Generally, the swerve is the most critical subsystem and should be prioritized accordingly, followed by the scoring mechanism. Endgame mechanisms are generally less critical.
- Issues regarding teleop should be prioritized over issues regarding autonomous.

### Open Alliance Posts

- All posts to the Open Alliance should be reviewed by a mentor before being posted unless a mentor has granted permission to post without review.
- Posts should include be interesting and informative.
- Posts should include pictures or videos when possible.
- Posts should be made after every competition and after major changes to the robot.
- Posts should be made regularly throughout the season, including during the off-season.
- All posts should be written on the team drive in a Google Doc.

### Competitions

- A new branch should be created for each competition where all changes made at the competition should be made.
- A release should be made of whatever code will be deployed to the robot at competition.
- All Limelight configs and other relevant files other than the code should be saved in the team drive and attached to the release.
- The code should be tested on the robot before the competition.
- 6328's Event Deploy extension should be used on to commit the code whenever it is deployed to the robot. Frequent manual commits are also fine.
- Code should be pushed after each day of the competition.
- Logs should be backed up after each day of the competition and the drive should be cleared in advance of the competition.

## Specifics

### Robot Container

- Robot container is where all subsystems are initialized and commands are scheduled or assigned to buttons.
- All button assignments and similar command scheduling should be done through the use of triggers. This makes the code more readable and allows multiple buttons to run the same command easily.
- Inline commands should be used when possible to avoid creating unnecessary command classes.
- If more complex logic is required, a command class should be created.
- Command factories inside of subsystems should be used instead of creating a bunch of instant commands in robot container.

### Swerve

- Swerve code is generally reused from year to year with minor modifications.
- Here is an overview of the control flow for swerve:

1. The drive method is passed the desired x and y velocities and the desired rotation velocity.
2. Desired velocities are passed into the setpoint generator.
3. The setpoint generator uses the swerve drive kinematics to calculate the desired wheel angles and speeds.
4. The setpoint generator desaturates the wheel speeds.
5. The setpoint generator sets the modules to setpoints that don't exceed the kinematic limits of the module.
6. Module states are optimized ensuring they don't rotate more than 90 degrees.
7. The descritize function is used to reduce skew.
8. The modules are set to their setpoints.

- Closed-loop rotation position control is done by putting a PID controller on the desired chassis rotation speed using the gyro as feedback.
- Swerve skew reduction is done by using the descritize function and a PID to either hold the last yaw or set the desired yaw velocity to zero.
- Deadzones should not be used on the base driver sticks.

### Scoring Mechanism

- The scoring mechanism should have some way to manually control the mechanism if the autonomous control fails.
- In the event that the autonomous control is necessary for the mechanism to be effective, manual control may be removed from the controller to free up more buttons.

### Endgame Mechanisms

- Endgame mechanisms should be assigned to buttons that won't easily be pressed early in the game.

### Teleop

- Teleop should include as much automation as possible to make the driver's job easier.

### Autonomous

- Autonomous should be made with the assumption that something will go wrong with the field or with an alliance partner and should be designed to be as robust as possible to those failures.
- Autonomous driving should be as fast as possible without sacrificing accuracy. Pose estimation and path following should be accurate enough that this should be close to the robot's top speed.

### Vision

- Field calibration should be a priority during load-in at competitions.
- Vision should be used to automate as much of the scoring mechanism as possible such as aligning the robot with the goal and picking up game pieces automatically.
- Functions of the robot should use reliable vision metrics when possible such as tx and ty because those are almost guaranteed to be accurate on the field.

### PID Loops

- Internal PID loops on the motor should be used when possible. They allow for high frequency control (up to 1000 hz) and better accuracy.
- Torque control should be used for all internal PID loops.
- For position, `DynamicMotionMagicTorqueCurrentFOC` should be used. Even if the profiling is not needed, the values should be set to the maximum that the subsystem can handle or the maximum that it can achieve while being accurate.
- For velocity, `MotionMagicVelocityTorqueCurrentFOC` should be used. The values should be set to the maximum that the subsystem can handle or the maximum that it can achieve while being accurate.
- Here are the meanings of the torque current PID gains:

**Position**:
| Gain: | Meaning: | Units: |
| --- | --- | --- |
| kP | Output per unit error in position. This gain is responsible for correcting the error between the desired and measured value. | $$\frac{Amps}{Rotations}$$ |
| kI | Output per unit of integrated error in position. This will make the controller have more of an effect if the target still isn't reached after a long time. | $$\frac{Amps}{Rotations*time}$$ |
| kD | Output per unit error in velocity. This can help reduce overshoot as it will slow down the output as it gets closer to the position. | $$\frac{Amps}{\frac{Rotations}{Second}}$$ |
| kS | Output to overcome friction. | $$Amps$$ |
| kV | Output to compensate for friction that scales with velocity (drag). | $$\frac{Amps}{\frac{Rotations}{Second}}$$ |
| kA | The motors current directly controls the torque (acceleration), this relates the amps to how it accelerates so it can hit desired acceleration targets. | $$\frac{Amps}{\frac{Rotations}{Second^2}}$$ |

**Velocity**:
| Gain: | Meaning: | Units: |
| --- | --- | --- |
| kP | Output per unit error in velocity. This gain is responsible for correcting the error between the desired and measured value. | $$\frac{Amps}{\frac{Rotations}{Second}}$$ |
| kI | Output per unit of integrated error in velocity. This will make the controller have more of an effect if the target still isn't reached after a long time. | $$\frac{Amps}{\frac{Rotations}{Second}*time}$$ |
| kD | Output per unit error in the derivative of velocity (acceleration). This can help reduce overshoot as it will compensate less as it gets closer to the desired velocity. | $$\frac{Amps}{\frac{Rotations}{Second^2}}$$ |
| kS | Output to overcome friction. | $$Amps$$ |
| kV | Output to compensate for friction that scales with velocity (drag). | $$\frac{Amps}{\frac{Rotations}{Second}}$$ |
| kA | The motors current directly controls the torque (acceleration), this relates the amps to how it accelerates so it can hit desired acceleration targets. | $$\frac{Amps}{\frac{Rotations}{Second^2}}$$ |

- Here is how to tune position gains:
  1. Start with `PositionTorqueCurrentFOC` instead of `DynamicMotionMagicTorqueCurrentFOC`.
  2. Start with a "low" kP (about 5).
  3. Target any positon.
  4. Increase kP until the motor reaches the target position and starts oscillating slightly, then back off.
  5. Change controller over to `DynamicMotionMagicTorqueCurrentFOC` with the current constants.
  6. Tune kS, kV, and kA like you would for velocity. The target velocity should be the maximum velocity the mechanism can reach while being accurate. The better tuned these are the faster you should be able to move in between positions. You may need to tune kP again after this.
  7. kI and kD are generally not needed, however, they can be added if necessary.
- Here is how to tune velocity gains:
  1. Start with `VelocityTorqueCurrentFOC` instead of `MotionMagicVelocityTorqueCurrentFOC`.
  2. Start with a "low" kP (about 5).
  3. Target a "low" velocity (about 20 rps works for flywheels).
  4. Increase kS until the motor reaches it's target velocity (start with a gain of 1 or so).
  5. Set target to a high velocity (about 60 rps for flywheels).
  6. Increase kV until measured velocity reaches target (initial gain of 1/target or so)
  7. Go back to “low” velocity, and decrease kS until measured reaches target.
  8. Repeat from step 5.
  9. Verify gains at other velocities.
  10. Increase kP until oscillation occurs, then back off.
  11. Change controller over to `MotionMagicVelocityTorqueCurrentFOC` with the current constants.
  12. Set a reasonable acceleration setpoint.
  13. Increase kA until the motor reaches the acceleration setpoint. If the motor is oscillating, decrease kP.
  14. Verify for multiple accelerations. Keep in mind, there is a physical limit to how fast the motor can accelerate based on the mechanism.
  15. kI and kD are generally not needed, however, they can be added if necessary.
- Internal PID loops also include a gravity feedforward gain (kG). This can be set up for either arm (cosine) or elevator (static) mechanisms. This value should be tuned so the mechanism can hold its position by itself.

### Logging

- All inputs from sensors and any outputs that are used should be logged. Ouputs can be added after and viewed in a replay if the logging is set up correctly.
- Inputs should be logged with auto logged inputs, outputs should be logged with auto logged output tags on the methods that set the outputs. Miscellanious data should be logged with `Logger.recordOutput()`.

### Simulation

- To be added
