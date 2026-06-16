"""Visual servoing module for fire-centroid tracking over thermal video.

Consumes the RTSP thermal stream from MediaMTX, thresholds the thermal image
using 2-sigma intensity isolation to extract the fire ROI, computes the ROI
centroid, and drives the drone laterally (via virtual stick) to centre the
fire in the camera frame.

Typical usage:
    from djiInterface import DJIInterface
    from visual_servo import VisualServo

    dji = DJIInterface("192.168.1.50")
    dji.startTelemetryStream()

    servo = VisualServo(dji, mediamtx_host="192.168.1.100")
    result = servo.run()   # blocks until converged or aborted
    # result.converged == True  →  fire is centred, ready to drop
"""

import cv2
import numpy as np
import time
import logging
from dataclasses import dataclass, field
from typing import Optional

log = logging.getLogger(__name__)


@dataclass
class ServoConfig:
    """Tuning knobs for the visual servo loop."""

    # --- PID gains (pixel-error → stick command) ---
    kp: float = 0.0012
    ki: float = 0.00002
    kd: float = 0.0004

    # --- Convergence ---
    pixel_threshold: int = 15          # px from centre to count as "on target"
    hold_duration: float = 2.0         # seconds centroid must stay within threshold
    max_duration: float = 60.0         # hard timeout for entire servo loop

    # --- Stick limits ---
    max_stick: float = 0.15            # cap stick output (conservative for close-range)

    # --- Loop timing ---
    loop_hz: float = 10.0              # target control-loop frequency

    # --- RTSP ---
    rtsp_port: int = 8554

    # --- Thermal thresholding ---
    sigma_factor: float = 2.0          # N in mean + N*std threshold
    min_contour_area: int = 200        # ignore blobs smaller than this (px²)

    # --- Debug visualisation ---
    show_debug: bool = False


@dataclass
class ServoResult:
    converged: bool = False
    reason: str = ""
    elapsed: float = 0.0
    final_error_px: float = float("inf")
    frames_processed: int = 0


class VisualServo:

    def __init__(
        self,
        dji,
        mediamtx_host: str = "localhost",
        config: Optional[ServoConfig] = None,
    ):
        self.dji = dji
        self.mediamtx_host = mediamtx_host
        self.cfg = config or ServoConfig()
        self._cap: Optional[cv2.VideoCapture] = None
        self._stop_requested = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(self) -> ServoResult:
        """Execute the visual servo loop. Blocks until converged / timed-out / aborted."""
        self._stop_requested = False
        result = ServoResult()

        prev_source = self.dji.getCameraStreamSource()
        log.info("Switching camera to INFRARED_CAMERA (was %s)", prev_source)
        self.dji.requestSetThermalStream()
        time.sleep(0.5)

        self.dji.requestSendEnableVirtualStick()
        time.sleep(0.3)

        try:
            self._open_stream()
            result = self._servo_loop()
        finally:
            self._send_stick(0.0, 0.0)
            self._close_stream()
            if prev_source and prev_source not in ("UNKNOWN", "INFRARED_CAMERA"):
                log.info("Restoring camera source to %s", prev_source)
                self.dji.requestSetCameraStreamSource(prev_source)

        return result

    def stop(self):
        """Request graceful stop from another thread."""
        self._stop_requested = True

    # ------------------------------------------------------------------
    # Stream management
    # ------------------------------------------------------------------

    def _rtsp_url(self) -> str:
        drone_ip = self.dji.IP_RC
        return f"rtsp://{self.mediamtx_host}:{self.cfg.rtsp_port}/{drone_ip}"

    def _open_stream(self):
        url = self._rtsp_url()
        log.info("Opening RTSP stream: %s", url)
        self._cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self._cap.isOpened():
            raise ConnectionError(f"Cannot open RTSP stream: {url}")
        log.info("RTSP stream opened")

    def _close_stream(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        if self.cfg.show_debug:
            cv2.destroyAllWindows()

    def _grab_frame(self) -> Optional[np.ndarray]:
        if self._cap is None or not self._cap.isOpened():
            return None
        ret, frame = self._cap.read()
        return frame if ret else None

    # ------------------------------------------------------------------
    # Thermal thresholding & centroid
    # ------------------------------------------------------------------

    def _find_fire_centroid(self, frame: np.ndarray):
        """2-sigma intensity threshold → largest blob centroid.

        Returns (cx, cy, mask) or (None, None, mask) if no fire detected.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        mean = np.mean(gray)
        std = np.std(gray)
        thresh_val = mean + self.cfg.sigma_factor * std
        thresh_val = min(thresh_val, 254.0)

        _, mask = cv2.threshold(gray, int(thresh_val), 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= self.cfg.min_contour_area]

        if not contours:
            return None, None, mask

        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None, None, mask
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx, cy, mask

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def _send_stick(self, right_x: float, right_y: float):
        """Send lateral velocity commands. leftX/leftY = 0 (no yaw/altitude)."""
        rx = np.clip(right_x, -self.cfg.max_stick, self.cfg.max_stick)
        ry = np.clip(right_y, -self.cfg.max_stick, self.cfg.max_stick)
        self.dji.requestSendStick(leftX=0, leftY=0, rightX=float(rx), rightY=float(ry))

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def _servo_loop(self) -> ServoResult:
        cfg = self.cfg
        dt = 1.0 / cfg.loop_hz

        integral_x = 0.0
        integral_y = 0.0
        prev_err_x = 0.0
        prev_err_y = 0.0
        on_target_since: Optional[float] = None

        t0 = time.time()
        frames = 0

        while not self._stop_requested:
            loop_start = time.time()
            elapsed = loop_start - t0

            if elapsed > cfg.max_duration:
                self._send_stick(0.0, 0.0)
                return ServoResult(
                    converged=False, reason="timeout", elapsed=elapsed,
                    frames_processed=frames)

            frame = self._grab_frame()
            if frame is None:
                log.warning("Frame grab failed, retrying")
                time.sleep(0.05)
                continue
            frames += 1

            h, w = frame.shape[:2]
            cx_frame = w / 2.0
            cy_frame = h / 2.0

            cx, cy, mask = self._find_fire_centroid(frame)

            if cx is None:
                # No fire blob detected — hold position
                self._send_stick(0.0, 0.0)
                on_target_since = None
                integral_x = 0.0
                integral_y = 0.0
                log.debug("No fire blob detected")
                self._debug_show(frame, mask, None, None, cx_frame, cy_frame, 0.0, 0.0)
                self._sleep_remainder(loop_start, dt)
                continue

            # Error: positive = fire is right/below centre
            err_x = (cx - cx_frame) / cx_frame   # normalised [-1, 1]
            err_y = (cy - cy_frame) / cy_frame

            # PID
            integral_x += err_x * dt
            integral_y += err_y * dt
            # Anti-windup clamp
            integral_x = np.clip(integral_x, -1.0, 1.0)
            integral_y = np.clip(integral_y, -1.0, 1.0)

            deriv_x = (err_x - prev_err_x) / dt
            deriv_y = (err_y - prev_err_y) / dt
            prev_err_x = err_x
            prev_err_y = err_y

            cmd_x = cfg.kp * err_x + cfg.ki * integral_x + cfg.kd * deriv_x
            cmd_y = cfg.kp * err_y + cfg.ki * integral_y + cfg.kd * deriv_y

            # DJI stick mapping with gimbal pointing down:
            #   rightX (+) → drone moves right → fire moves left in frame
            #   rightY (+) → drone moves forward → fire moves up in frame
            # So: positive pixel-X error (fire right of centre) → positive rightX
            #     positive pixel-Y error (fire below centre) → negative rightY (fly backward??)
            # Actually with nadir gimbal: pixel +Y = fire is ahead → need positive rightY (forward)
            # This depends on gimbal + heading. With gimbal -90 and camera top = drone forward:
            #   pixel +Y (down in image) = fire is further from drone forward direction
            # Standard mapping: rightY positive = forward pitch = drone moves forward
            # Image convention: +Y = down. Fire below centre means fire is ahead → fly forward → +rightY
            # BUT: with -90° gimbal, image top = drone forward. So fire BELOW centre = fire behind → -rightY
            # This is tricky. Using the safe assumption: gimbal -90, image top = forward.
            #   err_y > 0 (fire below centre) → fire is behind → need negative rightY
            self._send_stick(cmd_x, -cmd_y)

            # Convergence check
            pixel_err = np.sqrt((cx - cx_frame) ** 2 + (cy - cy_frame) ** 2)
            if pixel_err <= cfg.pixel_threshold:
                if on_target_since is None:
                    on_target_since = time.time()
                elif time.time() - on_target_since >= cfg.hold_duration:
                    self._send_stick(0.0, 0.0)
                    return ServoResult(
                        converged=True, reason="converged", elapsed=elapsed,
                        final_error_px=pixel_err, frames_processed=frames)
            else:
                on_target_since = None

            log.debug(
                "err=(%.3f,%.3f) cmd=(%.4f,%.4f) px_err=%.1f on_target=%.1fs",
                err_x, err_y, cmd_x, -cmd_y, pixel_err,
                (time.time() - on_target_since) if on_target_since else 0.0)

            self._debug_show(frame, mask, cx, cy, cx_frame, cy_frame, cmd_x, cmd_y)
            self._sleep_remainder(loop_start, dt)

        self._send_stick(0.0, 0.0)
        return ServoResult(converged=False, reason="stop_requested",
                           elapsed=time.time() - t0, frames_processed=frames)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _sleep_remainder(loop_start: float, dt: float):
        spent = time.time() - loop_start
        if spent < dt:
            time.sleep(dt - spent)

    def _debug_show(self, frame, mask, cx, cy, cx_frame, cy_frame, cmd_x, cmd_y):
        if not self.cfg.show_debug:
            return
        vis = frame.copy()
        h, w = vis.shape[:2]
        # Draw crosshair at frame centre
        cv2.drawMarker(vis, (int(cx_frame), int(cy_frame)), (0, 255, 0),
                       cv2.MARKER_CROSS, 30, 2)
        if cx is not None:
            cv2.circle(vis, (cx, cy), 10, (0, 0, 255), 2)
            cv2.line(vis, (int(cx_frame), int(cy_frame)), (cx, cy), (0, 255, 255), 1)
        cv2.putText(vis, f"cmd: ({cmd_x:+.3f}, {cmd_y:+.3f})",
                    (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("Visual Servo", vis)
        cv2.imshow("Thermal Mask", mask)
        cv2.waitKey(1)


# ----------------------------------------------------------------------
# Standalone test
# ----------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s %(name)s %(message)s")

    from djiInterface import DJIInterface

    drone_ip = sys.argv[1] if len(sys.argv) > 1 else ""
    mediamtx = sys.argv[2] if len(sys.argv) > 2 else "localhost"

    dji = DJIInterface(drone_ip)
    dji.startTelemetryStream()
    time.sleep(1)

    cfg = ServoConfig(show_debug=True)
    servo = VisualServo(dji, mediamtx_host=mediamtx, config=cfg)
    result = servo.run()

    print(f"\nResult: {result}")
    dji.stopTelemetryStream()
