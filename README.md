# Robot Odometry Display

Real-time visualization of robot position on FRC field using NetworkTables.

## What it does

Shows your robot moving on a field image with arrows indicating:
- 🔴 Red arrow = heading (where robot points)
- 🟢 Green arrow = velocity (where robot is moving)
- 🟣 Purple arrow = rotation (how fast robot is turning)

## Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Place your `IrishField.png` file in the same folder

3. Run the display:
```bash
python robot_localization_display.py
```

## Controls

- **H** - Hide/show telemetry overlay
- **ESC** - Quit
- Drag window edges to resize

## NetworkTables Format

Your robot code should publish to the `Pose` table:
- `X` - position in meters
- `Y` - position in meters  
- `Theta` - heading in radians
- `VX` - X velocity in m/s (optional)
- `VY` - Y velocity in m/s (optional)
- `Omega` - rotation speed in rad/s (optional)

The display automatically calculates velocities if not provided.
