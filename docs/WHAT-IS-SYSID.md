In FRC there are two main types of control: feedback and feedforward. Feedback-based controllers, like a PID, take sensor data and use it to calculate the output of a system, this is beneficial because it can adapt to unexpected change in a system. Take a thermostat for example, it uses a thermometer to determine how much the temperature of a house needs to be raised or lowered to reach it's target setpoint. Because of this, the thermostat will function in winter and summer, in a well insulated house and in a poorly insulated one. The feedback allows the system to dynamically adapt to change.

Feedback isn't perfect however, in a dynamic system, there is often a significant amount of lag when using a pure feedback controller, since it has no idea how the system will change over time. In addition, feedback controllers often have to decide between speed and accuracy. If you've ever tuned a PID controller you likely experienced this yourself, a proportional gain that is too high will reach the setpoint quickly, but is unstable and prone to oscilation, a gain that is too low will be slow to reach the setpoint or wont reach it at all. This is where feedforward controllers come in. A feedforward controller uses *no sensor data*, and instead calculates outputs based on a model of how your system will change over time. For example, stoplights don't actually know what traffic looks like at any given moment, instead, it switches from red to green based on a internal model derived from hundreds of hours of traffic simulation.

SysID is a tool that allows us to create a feedforward model of the subsystems in our robot, enabling fast and precise control. There are three main data points we need to calculate:

* `kS` - the static gain <p>
This is the energy required to overcome the static friction in your system, which is usually the result of minor mechanical imperfections in your system, such as belts or gears rubbing. This is constant, and exerts the same force on your system no matter the speed it is going at.

* `kV` - the velocity gain <p>
This gain accounts for the energy needed to maintain a certain velocity. This is different from `kS` because these forces *scale with velocity*, meaning that it takes more energy to maintain a higher velocity than it does a lower one.

* `kA` - the acceleration gain<p>
This gain accounts for the energy needed to transition between velocities. Like `kV`, this gain scales with acceleration.