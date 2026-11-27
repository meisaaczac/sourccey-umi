"""Portable Raspberry Pi Zero 2W daemon for wireless Sourccey UMI data collection.

Handles the optional portable hardware pack:
  - BNO085 / MPU6050 IMU over I2C (/dev/i2c-1) -> logs `imu.csv` at ~100 Hz
  - Physical Start/Stop pushbutton on GPIO 17 (active-low with internal pull-up)
  - Status LED / piezo buzzer feedback on GPIO 27 (idle / recording / saved / fault)

Works cleanly with `--mock` on any desktop OS for testing without Raspberry Pi GPIO/I2C.

Usage (on Raspberry Pi Zero 2W):
  python software/portable_node.py --out-dir ~/umi_sessions/episodes --imu bno085
Usage (local mock test):
  python software/portable_node.py --mock --out-dir ./tmp_ep --seconds 1
"""
import argparse
import csv
import json
import math
import threading
import time
from pathlib import Path


class ImuReader:
    """Reads 6/9-DoF IMU samples (BNO085, MPU6050, or mock) on the monotonic clock."""

    def __init__(self, kind: str = "mock"):
        self.kind = kind.lower()
        self._dev = None
        if self.kind == "bno085":
            import board
            import busio
            from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_ROTATION_VECTOR
            from adafruit_bno08x.i2c import BNO08X_I2C

            i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
            self._dev = BNO08X_I2C(i2c)
            self._dev.enable_feature(BNO_REPORT_ACCELEROMETER)
            self._dev.enable_feature(BNO_REPORT_GYROSCOPE)
            self._dev.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        elif self.kind == "mpu6050":
            import board
            import busio
            import adafruit_mpu6050

            i2c = busio.I2C(board.SCL, board.SDA)
            self._dev = adafruit_mpu6050.MPU6050(i2c)
        elif self.kind != "mock":
            raise ValueError(f"Unsupported IMU kind: {kind}")

    def read_sample(self) -> tuple[float, float, float, float, float, float, float, float, float, float]:
        """Return (ax, ay, az, gx, gy, gz, qx, qy, qz, qw)."""
        if self.kind == "bno085":
            ax, ay, az = self._dev.acceleration
            gx, gy, gz = self._dev.gyro
            qx, qy, qz, qw = self._dev.quaternion
            return (ax, ay, az, gx, gy, gz, qx, qy, qz, qw)
        if self.kind == "mpu6050":
            ax, ay, az = self._dev.acceleration
            gx, gy, gz = self._dev.gyro
            return (ax, ay, az, gx, gy, gz, 0.0, 0.0, 0.0, 1.0)
        # Mock sample (gravity along +z, slight gentle sway)
        t = time.monotonic()
        return (0.05 * math.sin(t), 0.0, 9.81, 0.0, 0.01 * math.cos(t), 0.0, 0.0, 0.0, 0.0, 1.0)


class ImuLoggerThread(threading.Thread):
    def __init__(self, imu: ImuReader, hz: float = 100.0):
        super().__init__(daemon=True)
        self.imu = imu
        self.dt = 1.0 / hz
        self.rows: list[tuple] = []
        self.stop = threading.Event()
        self.errors = 0

    def run(self):
        nxt = time.monotonic()
        while not self.stop.is_set():
            t_ns = time.monotonic_ns()
            try:
                s = self.imu.read_sample()
                self.rows.append((t_ns, *(round(float(v), 5) for v in s)))
            except Exception:
                self.errors += 1
            nxt += self.dt
            time.sleep(max(0.0, nxt - time.monotonic()))


class FeedbackController:
    """Drives status LED / piezo buzzer on GPIO 27 (or logs state in mock mode)."""

    def __init__(self, pin: int = 27, mock: bool = False):
        self.pin = pin
        self.mock = mock
        self.state = "idle"
        self._gpio = None
        if not mock:
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
                self._gpio = GPIO
            except Exception:
                self.mock = True

    def set_state(self, state: str):
        """States: idle, recording, saved, fault."""
        self.state = state
        if self.mock or self._gpio is None:
            return
        if state == "recording":
            self._gpio.output(self.pin, self._gpio.HIGH)
        elif state == "saved":
            for _ in range(2):
                self._gpio.output(self.pin, self._gpio.HIGH)
                time.sleep(0.08)
                self._gpio.output(self.pin, self._gpio.LOW)
                time.sleep(0.08)
        elif state == "fault":
            for _ in range(3):
                self._gpio.output(self.pin, self._gpio.HIGH)
                time.sleep(0.12)
                self._gpio.output(self.pin, self._gpio.LOW)
                time.sleep(0.12)
        else:
            self._gpio.output(self.pin, self._gpio.LOW)


class ButtonTrigger:
    """Physical tactile pushbutton on GPIO 17 (active-low, debounced)."""

    def __init__(self, pin: int = 17, debounce_s: float = 0.25, mock: bool = False):
        self.pin = pin
        self.debounce_s = debounce_s
        self.mock = mock
        self._last_press = 0.0
        self.is_recording = False
        self._gpio = None
        if not mock:
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                self._gpio = GPIO
            except Exception:
                self.mock = True

    def handle_press(self, now: float | None = None) -> str | None:
        """Toggle recording state with debounce. Returns 'start', 'stop', or None."""
        now = time.monotonic() if now is None else now
        if now - self._last_press < self.debounce_s:
            return None
        self._last_press = now
        self.is_recording = not self.is_recording
        return "start" if self.is_recording else "stop"


def write_imu_csv(out_path: str | Path, rows: list[tuple]) -> None:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_monotonic_ns", "ax", "ay", "az", "gx", "gy", "gz", "qx", "qy", "qz", "qw"])
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description="Portable Raspberry Pi Zero 2W node (IMU + Button + LED/Buzzer)")
    ap.add_argument("--out-dir", default="umi_sessions/portable_imu")
    ap.add_argument("--imu", choices=("bno085", "mpu6050", "mock"), default="bno085")
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--seconds", type=float, default=2.0, help="Duration for mock/one-shot capture")
    ap.add_argument("--mock", action="store_true")
    args = ap.parse_args()

    imu = ImuReader("mock" if args.mock else args.imu)
    fb = FeedbackController(mock=args.mock)
    btn = ButtonTrigger(mock=args.mock)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    fb.set_state("recording")
    t0 = time.monotonic_ns()
    logger = ImuLoggerThread(imu, hz=args.hz)
    logger.start()
    time.sleep(args.seconds)
    logger.stop.set()
    logger.join(2)
    fb.set_state("saved")
    write_imu_csv(out_dir / "imu.csv", logger.rows)
    summary = {
        "imu": imu.kind,
        "samples": len(logger.rows),
        "errors": logger.errors,
        "t_start_monotonic_ns": t0,
        "button_pin": btn.pin,
        "feedback_pin": fb.pin,
    }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
