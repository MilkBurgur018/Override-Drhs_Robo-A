## 0. Our globals at a glance

| Object | Type | What it is |
|---|---|---|
| `chassis` | `lemlib::Chassis` | The whole drivetrain + odometry. **Use this for movement.** |
| `left_mg` / `right_mg` | `pros::MotorGroup` | Drive motors (ports 12,13 left / 2,3 right, green cart) |
| `lift` | `pros::MotorGroup` | Lift motors (ports 1 reversed, 11) |
| `claws` | `pros::adi::Pneumatics` | Pneumatic claw on ADI port `A` |
| `imu` | `pros::Imu` | Inertial sensor, port 10 |
| `para_rot_l`, `para_rot_r`, `the_hozzy_one` | `pros::Rotation` | Tracking wheel sensors (19, 21, 20) |
| `master` | `pros::Controller` | Driver controller (not used in auton) |

### Coordinate system (IMPORTANT)

```
            +Y  (0°)
             ^
             |
 270° <------+------> +X (90°)
             |
            180°
```

- **Units:** x and y are in **inches**. Angles (theta) are in **degrees**.
- **0° = facing +Y (away from the driver wall), and angles increase CLOCKWISE.** So 90° = facing +X (right), 180° = facing backward, 270° (or -90°) = facing left.
- `(0, 0)` is wherever you tell the robot it starts with `chassis.setPose(...)`.
- All positions are **field-absolute**, not relative to where the robot currently is. `moveToPoint(0, 24, ...)` means "go to the spot at y = 24", not "drive 24 in".

---

## 1. Auton setup (do this FIRST in every auton)

```cpp
chassis.setPose(0, 0, 0);          // setPose(x [in], y [in], theta [deg]); tell the robot where it starts on the field. Do this at the start of EVERY auton.
chassis.setPose(-48, -60, 90);     // example: starting at x=-48in, y=-60in, facing right (90°)
```

`chassis.calibrate();` goes in `initialize()` (not in auton). It calibrates the IMU and takes ~3 seconds. Don't move the robot while it runs.

---

## 2. LemLib movement functions

### Common inputs used by all of these

| Input | Unit | Meaning |
|---|---|---|
| `x`, `y` | inches | Target field position |
| `theta` | degrees | Target heading (0° = +Y, clockwise positive) |
| `timeout` | milliseconds | Max time the move is allowed to take before it gives up and moves on. Always set this a bit longer than you think you need. |
| `maxSpeed` | 0 to 127 | Speed cap (127 = full speed) |
| `minSpeed` | 0 to 127 | Minimum speed. Setting it above 0 lets you chain moves without stopping |
| `earlyExitRange` | inches (or degrees for turns) | Ends the move early once the robot is this close to the target. Use with `minSpeed` for smooth, non-stopping chains |
| `async` | bool | `true` (default) = code continues immediately while the robot moves. `false` = code waits until the move finishes |

> **Default is `async = true`!** That means if you write two moves back to back, the second one cancels the first. Either pass `false` as the last argument or call `chassis.waitUntilDone();` between moves. See section 3.

---

### 2.1 Drive to a point

```cpp
chassis.moveToPoint(x, y, timeout);                       // x [in], y [in], timeout [ms]; drive in a straight-ish line to a field position. Robot ends facing whichever way it traveled.
chassis.moveToPoint(0, 24, 3000);                         // example: go to (0, 24), max 3 seconds
chassis.moveToPoint(0, 24, 3000, {.forwards = false});    // forwards [bool]; false = drive there BACKWARDS (robot's back faces the target)
chassis.moveToPoint(0, 24, 3000, {.maxSpeed = 80});       // maxSpeed [0-127]; slower, more controlled
chassis.moveToPoint(0, 24, 3000, {.forwards = true, .maxSpeed = 100, .minSpeed = 30, .earlyExitRange = 4}); // all options at once; minSpeed [0-127], earlyExitRange [in]
```

### 2.2 Drive to a point AND end at a specific heading (curved path)

```cpp
chassis.moveToPose(x, y, theta, timeout);                  // x [in], y [in], theta [deg], timeout [ms]; drive to a position and finish facing a specific direction. Follows a smooth curve (boomerang controller).
chassis.moveToPose(24, 24, 90, 4000);                      // example: go to (24, 24) and end facing right
chassis.moveToPose(24, 24, 90, 4000, {.forwards = false}); // forwards [bool]; false = drive there in reverse
chassis.moveToPose(24, 24, 90, 4000, {.lead = 0.4});       // lead [0 to 1, no unit]; how curvy the path is. Higher = wider curve. Default 0.6
chassis.moveToPose(24, 24, 90, 4000, {.horizontalDrift = 8, .lead = 0.6, .maxSpeed = 90, .minSpeed = 0, .earlyExitRange = 0}); // horizontalDrift [in] is already set by our drivetrain (2), only override if needed
```

### 2.3 Turn in place

```cpp
chassis.turnToHeading(theta, timeout);                     // theta [deg], timeout [ms]; spin in place to face an absolute field heading
chassis.turnToHeading(90, 1000);                           // example: face right (90°)
chassis.turnToHeading(180, 1000, {.direction = AngularDirection::CW_CLOCKWISE});          // direction; force the turn direction (CW_CLOCKWISE or CCW_COUNTERCLOCKWISE). Default AUTO = shortest way
chassis.turnToHeading(90, 1000, {.maxSpeed = 70, .minSpeed = 0, .earlyExitRange = 5});    // maxSpeed [0-127], minSpeed [0-127], earlyExitRange [deg]

chassis.turnToPoint(x, y, timeout);                        // x [in], y [in], timeout [ms]; spin in place to FACE a field position (without driving to it)
chassis.turnToPoint(24, 24, 1000);                         // example: face the point (24, 24)
chassis.turnToPoint(24, 24, 1000, {.forwards = false});    // forwards [bool]; false = turn so the BACK of the robot faces the point
chassis.turnToPoint(24, 24, 1000, {.maxSpeed = 70, .minSpeed = 0, .earlyExitRange = 5});  // maxSpeed [0-127], minSpeed [0-127], earlyExitRange [deg]
```

### 2.4 Swing turns (pivot around one side of the drivetrain)

One side of the drivetrain stays locked and the other side swings. Good for tight, quick direction changes.

```cpp
chassis.swingToHeading(theta, lockedSide, timeout);        // theta [deg], lockedSide [DriveSide::LEFT or DriveSide::RIGHT], timeout [ms]; swing to an absolute heading while locking one side
chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 1000); // example: left side locked, right side swings the robot to face 90°
chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 5}); // all options

chassis.swingToPoint(x, y, lockedSide, timeout);           // x [in], y [in], lockedSide [DriveSide::LEFT/RIGHT], timeout [ms]; swing to FACE a field position
chassis.swingToPoint(24, 24, lemlib::DriveSide::RIGHT, 1000); // example
chassis.swingToPoint(24, 24, lemlib::DriveSide::RIGHT, 1000, {.forwards = false, .direction = AngularDirection::AUTO, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 5}); // all options
```

### 2.5 Follow a path (Path.jerryio)

```cpp
// 1. In globals.cpp (or at the top of your auton file):  ASSET(my_path_txt);   // file must be called my_path.txt in the /static folder
chassis.follow(my_path_txt, lookahead, timeout);           // path [asset], lookahead [in], timeout [ms]; follow a pre-made path using pure pursuit
chassis.follow(my_path_txt, 15, 4000);                     // example: lookahead 15in, max 4 seconds. Bigger lookahead = smoother but cuts corners. Smaller = tighter but shakier
chassis.follow(my_path_txt, 15, 4000, false);              // forwards [bool]; false = follow the path in reverse
```

---

## 3. Chaining moves & waiting

Because moves are async by default, use these to sequence things properly.

```cpp
chassis.waitUntilDone();      // blocks (waits) until the current chassis movement is finished
chassis.waitUntil(12);        // waitUntil(dist [in]); blocks until the robot has traveled this many inches in the current move. Great for running the lift/claw mid-drive. For turns, the unit is degrees.
chassis.cancelMotion();       // stops the current chassis movement immediately
chassis.cancelAllMotions();   // stops ALL queued chassis movements
chassis.isInMotion();         // returns true if the chassis is currently executing a movement
```

**Pattern 1: run something halfway through a drive**

```cpp
chassis.moveToPoint(0, 36, 3000);   // start driving (async)
chassis.waitUntil(24);              // wait until we've gone 24 in
claws.extend();                     // open/close claw mid-drive
chassis.waitUntilDone();            // now wait for the drive to finish
```

**Pattern 2: just make it blocking** (simplest for beginners, pass `false` as the last argument)

```cpp
chassis.moveToPoint(0, 36, 3000, {}, false);   // {} = default options, false = NOT async (waits until done)
chassis.turnToHeading(90, 1000, {}, false);
```

---

## 4. Reading the robot's position

```cpp
lemlib::Pose pose = chassis.getPose();    // returns the robot's current pose; pose.x [in], pose.y [in], pose.theta [deg]
float x = chassis.getPose().x;            // inches
float y = chassis.getPose().y;            // inches
float heading = chassis.getPose().theta;  // degrees
imu.get_heading();                        // returns the IMU heading [0-360 deg]
imu.get_rotation();                       // returns total IMU rotation [deg], does not wrap at 360
pros::lcd::print(0, "X: %f", chassis.getPose().x);   // print to the brain screen, line 0 (needs pros::lcd::initialize() in initialize())
```

---

## 5. Drivetrain raw control (use sparingly)

These bypass LemLib's PID but odometry still tracks them. Use for shoving against a wall/goal, or when you need a dumb "drive for X time".

```cpp
left_mg.move(127);              // move(power [-127 to 127]); spin left side at this power (negative = backwards)
right_mg.move(127);             // same for right side. Both at 127 = full speed forward
left_mg.move(0);                // stop the left side (it will coast/brake depending on brake mode)
right_mg.move(0);               // stop the right side

// Example: shove forward for 1 second, then stop
left_mg.move(80);
right_mg.move(80);
pros::delay(1000);              // delay(time [ms])
left_mg.move(0);
right_mg.move(0);

chassis.tank(127, 127);         // tank(leftPower [-127 to 127], rightPower [-127 to 127]); same as above but goes through LemLib
chassis.arcade(100, 0);         // arcade(throttle [-127 to 127], turn [-127 to 127]); throttle = forward/back, turn = rotation (positive = clockwise)
chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);   // setBrakeMode(mode); BRAKE_COAST / BRAKE_BRAKE / BRAKE_HOLD. Hold actively resists being pushed.
```

> After raw moves, call `chassis.setPose(...)` or `chassis.turnToHeading(...)` to re-sync if the robot slipped or hit something.

---

## 6. Lift (`lift` motor group)

```cpp
lift.move(127);                       // move(power [-127 to 127]); run the lift at this power. Positive/negative = up/down depending on wiring. 0 = stop
lift.move(0);                         // stop the lift
lift.move_velocity(100);              // move_velocity(velocity [RPM, max 200 for green cart]); run at a set speed, more consistent than move()
lift.move_voltage(12000);             // move_voltage(voltage [millivolts, -12000 to 12000]); run at a set voltage
lift.move_absolute(500, 100);         // move_absolute(position [deg], velocity [RPM]); go to a position relative to the last zero point, then hold. Does NOT wait. Position is measured from where lift was last tared
lift.move_relative(200, 100);         // move_relative(distance [deg], velocity [RPM]); move this far from the CURRENT position. Does NOT wait
lift.tare_position();                 // sets the current lift position as 0 (do this when the lift is fully down at the start)
lift.get_position();                  // returns the current lift position [deg] (of the first motor in the group)
lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);   // makes the lift hold its position when stopped (use this so it doesn't fall)
lift.brake();                         // stop the lift using the current brake mode

// Example: raise lift to a position and wait until it's there (within 10 degrees)
lift.move_absolute(500, 100);
while (std::abs(lift.get_position() - 500) > 10) {
    pros::delay(10);                  // always delay inside loops!
}
```

---

## 7. Claws (`claws` pneumatics, ADI port A)

```cpp
claws.extend();         // pushes the piston out (extends the cylinder)
claws.retract();        // pulls the piston back in
claws.toggle();         // flips the piston to whatever it's not currently
claws.is_extended();    // returns true if currently extended, false if retracted
```

> Which one is "open" vs "closed" depends on how the claw is plumbed. Test it and write it here: **extend = ______ , retract = ______**

---

## 8. Waiting / timing

```cpp
pros::delay(500);       // delay(time [ms]); pause the auton for this long. 1000 ms = 1 second. Does NOT stop motors that are already running
```

---

## 9. Full example auton

```cpp
void my_first_auton() {
    // --- Setup ---
    chassis.setPose(-48, -60, 90);                        // start at (-48, -60) facing right
    claws.retract();                                      // start with claw in known state

    // --- Drive forward & grab something ---
    chassis.moveToPoint(-24, -60, 2000, {.maxSpeed = 90}, false);   // false = wait until done
    claws.extend();                                       // grab
    pros::delay(300);                                     // give the claw time to close

    // --- Turn and score ---
    chassis.turnToHeading(0, 1000, {}, false);            // face +Y (away from driver wall)
    chassis.moveToPoint(-24, -24, 2000, {}, false);       // drive forward 36 in
    lift.move_absolute(500, 100);                         // raise lift (doesn't wait)
    pros::delay(800);
    claws.retract();                                      // release

    // --- Back out ---
    chassis.moveToPoint(-24, -48, 2000, {.forwards = false}, false);   // reverse
    lift.move_absolute(0, 100);                           // lower lift
}
```

---

## 10. Tips & rules of thumb

1. **Always `setPose` at the start of auton** with the robot's real starting spot.
2. **Always give a timeout** that's a bit longer than needed. A move that never finishes will eat your entire auton.
3. **Remember `async` is true by default.** Use `waitUntilDone()` or pass `false` to make moves wait for each other.
4. **Angles are clockwise-positive, 0° = +Y.** If the robot turns the wrong way, check this first.
5. **Everything is absolute field coordinates**, not "go forward 24 in". To drive a relative distance, compute it from `chassis.getPose()` or just use the target coordinate.
6. **`pros::delay()` inside loops is mandatory**, or the brain will freeze.
7. **Test slow first** (`maxSpeed = 60`), then speed up.

---

## Heads-up on a few things in `globals.cpp`

These aren't movement functions, but they will directly affect how accurate every command above is:

- **Tracking wheel offsets:** the comment says the vertical wheels are 3" to each side and 2.5" behind center, and the horizontal wheel is at center, but the code uses `-2.5` for both vertical wheels and `0` for the horizontal one. LemLib's vertical-wheel offset is the *sideways* distance from the tracking center (left side negative, right side positive), so you probably want something like `-3` / `+3` there. Double-check against the LemLib docs for the exact sign convention for the horizontal wheel too.
- **Drivetrain RPM:** `360` is marked PLACEHOLDER. Green cartridge motors are 200 RPM, so if the drive is geared 1:1 this should be `200`; otherwise it's `200 × (driven teeth / driving teeth)`. Likewise confirm the track width (`13.5`) and wheel type.
- **Pose units:** if anything is off by a constant scale factor (e.g. asking for 24" and getting 20"), the wheel diameter or tracking-wheel diameter is the usual culprit.
