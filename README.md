# G-Zilla Motion Controller

Highly flexible multiple axis motion controller that accepts G code commands and controls multiple motors with STEP and DIR signals.

When it looks like it has already been done,
but something was always off.

I wrote this because I got tired of trying to adapt other people's solutions to my simple (?) needs.
Maybe this is due to me not willing to adapt to other people's code, or ignorance, or nobody having the same goal as me.

## What this code is for:

- Receiving G code commands - G code like you see in 3D printer firmwares. List of codes below.
- Applying all safety measures I could think of to prevent out-of-bounds travel or unexpected conditions.
- Driving any (reasonable) number of notion axes using Step and Direction signals, with no position feedback
- Performing coordinated moves with all motors
- Keeping an eye out for motor error input, emergency stop input, and (fun feature) a "Freeze motion" input

## Why I wrote it:

Despite my best effort, I have not found a motion controller code that is:
- Written for Arduino (accessible for me)
- Accepts G code as we know it from 3D printer firmwares as input
- Hardware agnostic (does NOT use low-level hardware-specific trickery to do the impossible)
- Runs on TEENSY 4.1 (which has enough IO and 600 MHz clock to sustain enough axes of motion)
- Supports more than 6 axes of motion easily (6 axis robot arm + track)
- Adaptable to slap on an inverse kinematics layer to control a robot arm (yes, that is in the works)

If you know a controller that can do all this - DM me right now and I will happily use that.

## Limitations:
- Limited to only the commands necessary and required for robot arm movement (easy to add new commands)
- Limited to a specific G code format: G0 X25 Y-9 Z9 U300 V2 W-200 F4000 (omit what isn't needed)
- Serial feedback should be standardised and improved to be accepted by existing G code senders
- Inverse kinematics to be implemented: I have the math aready (big progress)

## Next level:
- Add inverse kinematics into this code, make IK calculationsbefore motor moves and forward kinematics for getting position
- Switch between direct joint control and IK control by G20 (direct joint pos) and G21 (IK)
