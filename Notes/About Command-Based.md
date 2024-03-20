- [Command-Based Programming Digest](#command-based-programming-digest)
  - [Subsystems and Commands](#subsystems-and-commands)
- [Commands](#commands)
  - [Structure](#structure)

-------------------
## Command-Based Programming Digest

The command-based paradigm is an example of declarative programming.

> Example: specify that the robot should perform an action when a condition is true: 
```java
new Trigger(condition::get).onTrue(Commands.runOnce(() -> piston.set(DoubleSolenoid.Value.kForward)));
```
In constrast of:
```java
if(condition.get()) {
  if(!pressed) {
    piston.set(DoubleSolenoid.Value.kForward);
    pressed = true;
  }
} else {
  pressed = false;
}
```

### Subsystems and Commands
Two core abstractions:
- Subsystems: represent independently-controlled collections of robot hardware (such as motor controllers, sensors, pneumatic actuators, etc.) that operate together.
- Commands: represent actions the robot can take. Commands run when scheduled, until they are interrupted or their end condition is met. 

Important concepts:
- Command Compositions: simple pieces build complex commands

## Commands

### Structure
- Initialization
- Execution
- Ending

