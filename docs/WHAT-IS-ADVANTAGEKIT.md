> [!NOTE]
> The following is intended to be a begginer-friendly introduction to AdvantageKit, NOT a comprehensive guide, for a full, up-to-date guide, check out the [official AKit docs.](https://github.com/Mechanical-Advantage/AdvantageKit) 

## Why is AdvantageKit?
AdvantageKit is a programming framework that allows us to log several important bits of data on the robot for later use; whenever we turn our robot on, it immediately begins logging various types of data, such as motor outputs, sensor readings, and other critical metrics. For example, in our 2024 robot, sometimes the intake wouldn't run when it was supposed to, this could have been caused by a variety of reasons, but AdvantageKit allowed us to look through the data logs and trace the issue back to a sensor that would misfire ocassionally.

## How is AdvantageKit?
AdvantageKit logs data through two primary methods: explicit logging statements in code and via the use of an IO layer.

### Explicit logging statements
This is the simplest way to log values, simply type `Logger.recordOutput(name, data);` where `name` is the name the data will be saved under, and `data` is the actual value you want to record. You can record several data types, like numbers, strings, Pose2Ds, custom enums, etc. 

### The IO layer
The IO (Input-Output) layer is the most important part of Advantagekit. AKit splits your subsystems into three layers: the hardware layer, the IO layer, and the control layer.

The hardware layer contains the code that talks directly to the robot and will be named `SubsytemIOHardware.java`, where `Hardware` is the name of the specific device you are using to control the subsystem, for example an elevator controlled by a Falcon 500 motor would be called `ElevatorIOFalcon.java`. All sensor readings and motor commands are done only through this layer.

The control layer is the public-facing side of the code, and will be named something like `SUBSYSTEM.java`. This contains all the logic required to control a subsystem, for example, the method to set a motor's position will be in the hardware layer, but the method to calculate the position a motor should be at based on the current position of the robot on the field will be in the control layer. ALL control logic HAS to be in the control layer, meaning you could have a method in the control layer that just calls a method in the hardware layer and does nothing else, while this may seem pointless since you could just call the method from the hardware layer directly, it is important that the code is structured in this way to ensure that the IO layer functions properly

> [!WARNING]
> You should NEVER have to instantiate a piece of hardware in the control layer! For example, if you find yourself typing something along the lines of `new TalonFX(Constants.TalonPort)` inside of the control layer, you have likely done something wrong! Acessing hardware directly inside of the control layer instead of through the hardware layer will completely mess up logging capabilities

The IO layer sits in between the hardware layer and the control layer