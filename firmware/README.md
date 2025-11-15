# 3D Scanner Firmware

Firmware for 3D scanner using TF-Luna LiDAR and 2 stepper motors (X and Y axes).

## Hardware Configuration

### Stepper Motors (CNC Shield style)
- **Motor X (Theta/Rotation)**:
  - STEP: Pin 2
  - DIR: Pin 5
  - ENABLE: Pin 8

- **Motor Y (Z-axis/Vertical)**:
  - STEP: Pin 3
  - DIR: Pin 6
  - ENABLE: Pin 8 (shared with X on some shields)

### TF-Luna LiDAR
- **I2C Connection**:
  - SDA: Pin A4 (Arduino default)
  - SCL: Pin A5 (Arduino default)
  - I2C Address: 0x10 (default)

## Building and Uploading

### Using PlatformIO

1. Install PlatformIO:
   - VS Code: Install PlatformIO IDE extension
   - Or use PlatformIO CLI

2. Build the project:
```bash
cd firmware
pio run
```

3. Upload to Arduino:
```bash
pio run -t upload
```

4. Monitor serial output:
```bash
pio device monitor
```

### Using Arduino IDE

If you prefer Arduino IDE, you can copy the contents of `src/main.cpp` to a new Arduino sketch. Make sure to:
- Install Arduino framework
- Include Wire library (built-in)

## Serial Commands

The firmware responds to the following serial commands:

- `START` - Begin scanning
- `STOP` - Stop scanning
- `HOME` - Return to home position
- `TEST` - Test TF-Luna reading

## Scan Data Format

Scan data is sent via Serial in the format:
```
LAYER,STEP,DISTANCE
```

Where:
- LAYER: Current Z layer (0 to max layers)
- STEP: Current rotation step (0 to steps per revolution)
- DISTANCE: Distance reading in cm (or -1 for invalid)

Layer delimiters are sent as: `9999`

## Configuration

Edit the configuration section at the top of `main.cpp` to adjust:
- Motor step counts
- Z-axis travel distance
- Scan delay
- Serial baud rate

