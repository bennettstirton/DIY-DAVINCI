import math

# =============================================================================
# CONFIG — tune all parameters here
# =============================================================================

# --- Loop timing ---
CONTROL_LOOP_MS  = 40
CONTROL_LOOP_SEC = CONTROL_LOOP_MS / 1000.0

# --- Microstep settings ---
# IMPORTANT: These values must match your physical driver DIP switch settings.
#   DM556T  (Pitch):  valid values 1, 2, 4, 8, 16, 32
#   TB6600  (Roll):   valid values 1, 2, 4, 8, 16, 32
#   TB6600  (Linear): valid values 1, 2, 4, 8, 16, 32
PITCH_MICROSTEPS  = 16
ROLL_MICROSTEPS   = 4
LINEAR_MICROSTEPS = 4    # set TB6600 DIP switches to match

# --- Derived: steps per revolution ---
PITCH_STEPS_PER_REV  = 200 * PITCH_MICROSTEPS    # 16x → 3200 steps/rev
ROLL_STEPS_PER_REV   = 200 * ROLL_MICROSTEPS     #  4x →  800 steps/rev
LINEAR_STEPS_PER_REV = 200 * LINEAR_MICROSTEPS   #  4x →  800 steps/rev

# --- Linear axis geometry ---
# Cable spool diameter: 20mm  →  circumference = 20 * pi ≈ 62.83 mm/rev
# Total travel: LINEAR_MAX_MM (175mm)  →  175 / 62.83 ≈ 2.8 revolutions max
#
# NOTE: cable spool effective diameter grows slightly as cable layers accumulate.
# Over ~3 revolutions this is a small effect but means steps/mm drifts
# slightly near full extension. Acceptable for most applications.
LINEAR_SPOOL_DIAMETER_MM = 20.0
LINEAR_SPOOL_CIRC_MM     = math.pi * LINEAR_SPOOL_DIAMETER_MM   # ~62.83 mm/rev
LINEAR_STEPS_PER_MM      = LINEAR_STEPS_PER_REV / LINEAR_SPOOL_CIRC_MM
LINEAR_MAX_MM            = 175.0
LINEAR_MAX_STEPS         = int(LINEAR_MAX_MM * LINEAR_STEPS_PER_MM)

# --- Optical encoder (linear axis input) ---
#
# !! FILL THIS IN BEFORE RUNNING !!
# PPR (Pulses Per Revolution) is printed on the encoder body or its datasheet.
# Common values: 100, 200, 360, 600. Using 4x quadrature decoding, the actual
# resolution the code sees is PPR * 4 counts per revolution.
ENCODER_PPR = 600   # <-- UPDATE THIS to match your encoder

# Gear ratio between optical encoder and linear stepper.
# ENCODER_SCALE = 1.0 means: 1 full turn of the encoder → 1 full rev of the stepper.
# ENCODER_SCALE = 2.0 means: 0.5 turns of the encoder → 1 full rev of the stepper (faster/coarser).
# ENCODER_SCALE = 0.5 means: 2 full turns of the encoder → 1 full rev of the stepper (slower/finer).
# Start conservatively (0.5 or 1.0) and increase once motion feels right.
ENCODER_SCALE = 0.75

# Derived: how many stepper steps result from one encoder count
# (ENCODER_PPR * 4 because we decode all 4 edges per cycle = 4x resolution)
ENCODER_COUNTS_PER_REV  = ENCODER_PPR * 4
STEPS_PER_ENCODER_COUNT = (LINEAR_STEPS_PER_REV * ENCODER_SCALE) / ENCODER_COUNTS_PER_REV

# --- GPIO pin assignments ---
PITCH_STEP_PIN  = 14
PITCH_DIR_PIN   = 27
ROLL_STEP_PIN   = 26
ROLL_DIR_PIN    = 25
LINEAR_STEP_PIN = 13
LINEAR_DIR_PIN  = 15

# Main input: MPU6050 IMU (replaces the 2-axis analog joystick formerly on
# GPIO 34/35 — those wires now carry the IMU's SDA/SCL to the I2C bus below).
# The IMU shares the existing AS5600/TCA9548A I2C bus (GPIO 21/22) — its
# address (0x68) doesn't collide with the TCA9548A (0x70) or AS5600s (0x36,
# selected behind the mux).
IMU_ADDR = 0x68

# One-time calibration offsets — measured with armcontrolsetup.py Test D
# (run with zeroing skipped, note the raw reading at the working-neutral
# mount position, enter the correction needed to bring it to 0).
# calibrated_angle = raw_accel_angle + offset
IMU_PITCH_OFFSET_DEG = 0.0
IMU_ROLL_OFFSET_DEG  = -90.0

# Physical tilt range (degrees from calibrated neutral) that maps to full
# +/-1.0 command — i.e. the IMU equivalent of full joystick deflection.
IMU_PITCH_MAX_TILT_DEG = 30.0
IMU_ROLL_MAX_TILT_DEG  = 30.0

# Tilt within this many degrees of neutral reads as zero (prevents jitter
# at rest from being interpreted as a command).
IMU_DEADBAND_PITCH_DEG = 1.5
IMU_DEADBAND_ROLL_DEG  = 1.5

# Trim joystick (replaces 4 rocker switches, freeing GPIO 18 and 19)
# Wire X axis to GPIO 32, Y axis to GPIO 33 (both ADC1, safe for use).
TRIM_JOY_X_PIN = 32   # controls Pitch trim
TRIM_JOY_Y_PIN = 33   # controls Roll trim

# Optical encoder pins (previously linear rocker FWD/BWD)
# Wire via voltage divider if encoder runs on 5V (see wiring notes at top of file).
#   Channel A → R1(1kΩ) → junction → R2(2kΩ) → GND  ; junction → GPIO 16
#   Channel B → R1(1kΩ) → junction → R2(2kΩ) → GND  ; junction → GPIO 17
ENCODER_A_PIN = 16
ENCODER_B_PIN = 17

# Limit switch — wired between GPIO23 and GND (pull-up used, normally closed)
# Triggered when the carriage reaches the home (fully retracted) end.
LINEAR_LIMIT_PIN = 23

# E-stop — NC (normally closed) button between GPIO4 and GND (pull-up used).
# Normal: button closed → pin LOW.  E-stop pressed (or wire break): pin HIGH.
# Fail-safe: a broken wire triggers the stop, same as pressing the button.
ESTOP_PIN = 4

# Re-arm — NO (normally open) momentary button between GPIO5 and GND (pull-up used).
# Press after releasing the e-stop mushroom to resume without a full ESP32 reset.
REARM_PIN = 5

# --- I2C bus + TCA9548A multiplexer ---
# Both AS5600 encoders share one I2C bus via the TCA9548A multiplexer.
# TCA9548A wiring: VCC→3.3V, GND→GND, SDA→GPIO21, SCL→GPIO22, A0/A1/A2→GND (addr 0x70).
# Each AS5600 connects to its own TCA channel — no address collision.
AS5600_SDA_PIN       = 21
AS5600_SCL_PIN       = 22
AS5600_I2C_FREQ      = 100000
AS5600_ADDR          = 0x36
AS5600_RAW_ANGLE_REG = 0x0C
TCA9548A_ADDR        = 0x70
ROLL_TCA_CHANNEL     = 0    # roll  AS5600 on TCA channel 0
PITCH_TCA_CHANNEL    = 1    # pitch AS5600 on TCA channel 1

# Flip to True if encoder reads increasing angles in the wrong direction.
ROLL_ENCODER_INVERT   = False
PITCH_ENCODER_INVERT  = False
LINEAR_ENCODER_INVERT = False  # flip to True if extending input decreases encoder_position_steps

# Flip to True if the position counter moves in the wrong direction after
# LINEAR_INVERT_DIR is already set correctly for motor direction.
# These two flags are intentionally independent — motor direction and counter
# direction can be set separately without affecting each other or homing.
LINEAR_POSITION_INVERT = True

# Physical home offset. Jog to desired zero, note the printed angle, enter it here.
ROLL_ENCODER_OFFSET_DEG  = 0.0
PITCH_ENCODER_OFFSET_DEG = 0.0

# --- Position control scaling ---
ROLL_MAX_DEGREES  = 15.0 # was 45.0
PITCH_MAX_DEGREES = 15.0 # was 45.0

# --- PWM motor speed ---
PITCH_MAX_RPS  = 1.0    # rev/sec — NEMA 23 / DM556T
ROLL_MAX_RPS   = 10.0   # rev/sec — NEMA 17 + 10:1 gearbox
LINEAR_MAX_RPS = 2.0    # rev/sec — tune to taste; start conservatively

# --- Derived: frequency limits ---
PITCH_MAX_FREQ  = int(PITCH_MAX_RPS  * PITCH_STEPS_PER_REV)
ROLL_MAX_FREQ   = int(ROLL_MAX_RPS   * ROLL_STEPS_PER_REV)
LINEAR_MAX_FREQ = int(LINEAR_MAX_RPS * LINEAR_STEPS_PER_REV)

# Minimum PWM frequency — below this the motor stops entirely.
MIN_FREQ = 20

# --- PID gains (rotary axes only — linear is open-loop) ---
ROLL_KP       = 50.0
ROLL_KI       = 5.0
ROLL_KD       = 1.0
ROLL_KI_CLAMP = 500.0

PITCH_KP       = 50.0
PITCH_KI       = 5.0
PITCH_KD       = 3.0
PITCH_KI_CLAMP = 500.0

# --- Position deadbands ---
PITCH_POSITION_DEADBAND_DEG = 1.0
ROLL_POSITION_DEADBAND      = 1.0

# --- Trim joystick ADC config (must match physical joystick) ---
TRIM_JOY_MIN      = 200
TRIM_JOY_MAX      = 3895
TRIM_JOY_CENTRE   = 2048
TRIM_JOY_DEADBAND = 300
TRIM_JOY_INVERT_X = False
TRIM_JOY_INVERT_Y = False

# --- Jogging (trim joystick) ---
PITCH_JOG_RPS = 0.1 # Open loop. Does not factor in capstan reduction.
ROLL_JOG_RPS  = 1.0 # Open loop. Does not factor in capstan/gearbox reduction.

PITCH_JOG_FREQ = int(PITCH_JOG_RPS * PITCH_STEPS_PER_REV)
ROLL_JOG_FREQ  = int(ROLL_JOG_RPS  * ROLL_STEPS_PER_REV)

# --- Jog ramp rates ---
ROLL_JOG_ACCEL_RPS2 = 3.0
ROLL_JOG_DECEL_RPS2 = 3.0
ROLL_JOG_ACCEL_HZ   = max(1, int(ROLL_JOG_ACCEL_RPS2  * ROLL_STEPS_PER_REV  * CONTROL_LOOP_SEC))
ROLL_JOG_DECEL_HZ   = max(1, int(ROLL_JOG_DECEL_RPS2  * ROLL_STEPS_PER_REV  * CONTROL_LOOP_SEC))

PITCH_JOG_ACCEL_RPS2 = 1.0
PITCH_JOG_DECEL_RPS2 = 1.0
PITCH_JOG_ACCEL_HZ   = max(1, int(PITCH_JOG_ACCEL_RPS2 * PITCH_STEPS_PER_REV * CONTROL_LOOP_SEC))
PITCH_JOG_DECEL_HZ   = max(1, int(PITCH_JOG_DECEL_RPS2 * PITCH_STEPS_PER_REV * CONTROL_LOOP_SEC))

# Deceleration rate when optical encoder stops turning.
# Higher value = stops more abruptly. Lower = coasts to a stop.
LINEAR_ENC_DECEL_RPS2 = 4.0
LINEAR_ENC_DECEL_HZ   = max(1, int(LINEAR_ENC_DECEL_RPS2 * LINEAR_STEPS_PER_REV * CONTROL_LOOP_SEC))

# Minimum encoder counts per 40ms tick required to command linear motor motion.
# Filters out single-count EMI noise induced by pitch/roll stepper drivers on
# the encoder A/B wires. A real intentional input at even slow speed will
# produce many counts per tick; a noise spike typically produces only 1-2.
# Raise if phantom motion persists; lower if slow intentional inputs are ignored.
LINEAR_ENC_MIN_DELTA = 8

# --- Step direction polarity ---
PITCH_INVERT_DIR  = False
ROLL_INVERT_DIR   = False
LINEAR_INVERT_DIR = True   # flip if extend/retract are physically backwards

# --- Homing speed ---
# Keep this conservative — the carriage hits the switch at this speed.
LINEAR_HOMING_RPS  = 0.1
LINEAR_HOMING_FREQ = int(LINEAR_HOMING_RPS * LINEAR_STEPS_PER_REV)

# --- IMU input EMA smoothing ---
# Lower = smoother but more sluggish. Higher = more responsive but noisier.
EMA_ALPHA = 0.2

# =============================================================================
# DEMO MODE
# =============================================================================
# Set DEMO_MODE = True to run the preprogrammed motion sequence instead of
# responding to the IMU. Jog to the desired home position with the trim
# joystick before enabling, then reset the ESP32.

DEMO_MODE = False

# Degrees the arm travels in each direction for the axis sweep.
DEMO_PITCH_AMPLITUDE_DEG = 10.0
DEMO_ROLL_AMPLITUDE_DEG  = 10.0

# Radius (degrees) of the orbit circle at the end of the sequence.
DEMO_ORBIT_RADIUS_DEG = 10.0

# How fast the orbit spins — full circles per second.
DEMO_ORBIT_RPS = 0.15

# How long the arm dwells at each waypoint extreme before returning (ms).
DEMO_HOLD_MS = 1500

# How long the orbit runs before the sequence restarts (ms).
DEMO_ORBIT_DURATION_MS = 8000

# Speed cap for demo moves — fraction of PITCH/ROLL_MAX_FREQ.
# Lower = smoother and safer for filming; raise if moves look sluggish.
DEMO_SPEED_FRACTION = 0.6
