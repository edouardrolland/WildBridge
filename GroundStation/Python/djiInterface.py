import requests
import ast
import os
import re
import sys
import time
import cv2
import json
import socket
import threading

import math

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime

# Discovery Configuration
DISCOVERY_PORT = 30000
DISCOVERY_MSG = b"DISCOVER_WILDBRIDGE"
DISCOVERY_RESPONSE_PREFIX = "WILDBRIDGE_HERE:"

def discover_drone(timeout=5.0):
    """
    Discover WildBridge drone on the network using UDP broadcast.
    Returns the IP address of the first drone found, or None if not found.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(timeout)
    
    try:
        # Send broadcast
        sock.sendto(DISCOVERY_MSG, ('<broadcast>', DISCOVERY_PORT))
        print(f"Broadcasting discovery message on port {DISCOVERY_PORT}...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                data, addr = sock.recvfrom(1024)
                message = data.decode('utf-8')
                if message.startswith(DISCOVERY_RESPONSE_PREFIX):
                    drone_ip = message.split(":")[1]
                    print(f"Found WildBridge drone at {drone_ip}")
                    return drone_ip
            except socket.timeout:
                break
    except Exception as e:
        print(f"Discovery error: {e}")
    finally:
        sock.close()
    
    return None

# Aircraft state endpoint suffixes
# GETTER
EP_BASE = "/"
EP_SPEED = "/aircraft/speed"
EP_HEADING = "/aircraft/heading"
EP_ATTITUDE = "/aircraft/attitude"
EP_LOCATION = "/aircraft/location"
EP_GIMBAL_ATTITUDE = "/aircraft/gimbalAttitude"
EP_ALL_STATES = "/aircraft/allStates"
EP_STICK_VALUES = "/aircraft/rcStickValues"
EP_YAW_REACHED = "/status/yawReached"
EP_ALTITUDE_REACHED = "/status/altitudeReached"
EP_WP_REACHED = "/status/waypointReached"
EP_HOME_LOCATION = "/home/location"
EP_CAMERA_IS_RECORDING = "/status/camera/isRecording"
EP_CAPTURE_THERMAL_IMAGE = "/send/captureThermalImage"

# SETTER
# HTTP POST Command Endpoints (port 8080)
EP_STICK = "/send/stick"  # expects a formatted string: "<leftX>,<leftY>,<rightX>,<rightY>"
EP_ZOOM = "/send/camera/zoom"
EP_GIMBAL_SET_PITCH = "/send/gimbal/pitch"
EP_GIMBAL_SET_YAW = "/send/gimbal/yaw"  # !!! This is the yaw joint angle !!!
EP_TAKEOFF = "/send/takeoff"
EP_LAND = "/send/land"
EP_RTH = "/send/RTH"
EP_ENABLE_VIRTUAL_STICK = "/send/enableVirtualStick"
EP_ABORT_MISSION = "/send/abortMission"
EP_GOTO_WP = "/send/gotoWP"
EP_GOTO_YAW = "/send/gotoYaw"
EP_GOTO_WP_PID = "/send/gotoWPwithPID"
EP_GOTO_TRAJECTORY = "/send/navigateTrajectory"
EP_GOTO_ALTITUDE = "/send/gotoAltitude"
EP_CAMERA_START_RECORDING = "/send/camera/startRecording"
EP_CAMERA_STOP_RECORDING = "/send/camera/stopRecording"
EP_GOTO_TRAJECTORY_DJI_NATIVE = "/send/navigateTrajectoryDJINative"
EP_ABORT_DJI_NATIVE_MISSION = "/send/abort/DJIMission"
EP_SET_RTH_ALTITUDE = "/send/setRTHAltitude"
EP_DEACTIVATE_MANUAL_OVERRIDE = "/send/deactivateManualOverride"
EP_GET_MANUAL_OVERRIDE = "/get/isManualOverrideActive"
EP_LRF_MEASURE = "/send/lrf/measure"
EP_LIST_MEDIA = "/send/listMedia"
EP_DOWNLOAD_MEDIA_BY_NAME = "/send/downloadMediaByName"

# The bridge identifies the three co-aligned lenses of one H20T shutter by these
# exact names (no aliases). One shutter exposes all of them at once; the lens(es)
# you pass select which are downloaded.
LENS_KEYS = ("thermal", "wide", "zoom")


def canonical_lenses(lenses):
    """Normalize a lens selection into an ordered, de-duplicated list of lens names.

    Accepts a single name ("wide"), a list/tuple, or a comma/space-separated string
    ("thermal,zoom"). The only valid names are the members of LENS_KEYS — no aliases;
    matching is case-insensitive. Output order follows LENS_KEYS. Raises ValueError on
    an unknown lens name.
    """
    if isinstance(lenses, str):
        lenses = [part for part in lenses.replace(",", " ").split() if part]
    requested = set()
    for name in lenses:
        key = str(name).strip().lower()
        if key not in LENS_KEYS:
            raise ValueError(f"unknown lens {name!r}; valid: {', '.join(LENS_KEYS)}")
        requested.add(key)
    return [k for k in LENS_KEYS if k in requested]


# PID Tuning
EP_TUNING = "/send/gotoWPwithPIDtuning"
EP_PAYLOAD_DROP = "/send/drop"

#PID Tuninng
EP_TUNING = "/send/gotoWPwithPIDtuning"

# Thermal Image handling
SAVE_SUCCESS = "T_IMG_SAVE_SUCCESS"
SAVE_FAILURE = "T_IMG_SAVE_FAILURE"
CAP_FAILURE = "T_IMG_CAP_FAILURE"

class DJIInterface:
    """
    Interface for DJI drone control via HTTP commands (port 8080) and 
    TCP telemetry socket (port 8081).
    """
    
    def __init__(self, IP_RC=""):
        if not IP_RC:
            print("No IP provided, attempting to discover drone...")
            discovered_ip = discover_drone()
            if discovered_ip:
                self.IP_RC = discovered_ip
            else:
                print("Drone discovery failed.")
                self.IP_RC = ""
        else:
            self.IP_RC = IP_RC

        self.baseCommandUrl = f"http://{self.IP_RC}:8080"
        self.telemetryPort = 8081
        self.videoSource = f"rtsp://aaa:aaa@{self.IP_RC}:8554/streaming/live/1"
        
        # Telemetry state (updated via TCP socket)
        self._telemetry = {}
        self._telemetry_lock = threading.Lock()
        self._telemetry_socket = None
        self._telemetry_thread = None
        self._running = False

    def getVideoSource(self):
        if self.IP_RC == "":
            return ""
        return self.videoSource

    # ==================== Telemetry (TCP Socket on port 8081) ====================
    
    def startTelemetryStream(self):
        """
        Start receiving telemetry data via TCP socket connection.
        The drone sends JSON telemetry data continuously.
        """
        if self._running:
            return
        
        self._running = True
        self._telemetry_thread = threading.Thread(target=self._telemetry_receiver, daemon=True)
        self._telemetry_thread.start()
    
    def stopTelemetryStream(self):
        """Stop the telemetry stream and close the socket."""
        self._running = False
        if self._telemetry_socket:
            try:
                self._telemetry_socket.close()
            except:
                pass
        if self._telemetry_thread:
            self._telemetry_thread.join(timeout=2)
    
    def _telemetry_receiver(self):
        """Background thread that receives telemetry data from TCP socket."""
        buffer = ""
        while self._running:
            try:
                # Connect to telemetry server
                self._telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._telemetry_socket.settimeout(5.0)
                self._telemetry_socket.connect((self.IP_RC, self.telemetryPort))
                
                while self._running:
                    data = self._telemetry_socket.recv(4096)
                    if not data:
                        break
                    
                    buffer += data.decode('utf-8')
                    
                    # Process complete JSON objects (separated by newlines)
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            try:
                                telemetry = json.loads(line)
                                with self._telemetry_lock:
                                    self._telemetry = telemetry
                                    self._telemetry["timestamp"] = datetime.now().strftime(
                                        "%Y-%m-%d_%H-%M-%S.%f")
                            except json.JSONDecodeError:
                                pass
                                
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Telemetry connection error: {e}")
                import time
                time.sleep(1)  # Wait before reconnecting
            finally:
                if self._telemetry_socket:
                    try:
                        self._telemetry_socket.close()
                    except:
                        pass
    
    def getTelemetry(self):
        """
        Get the latest telemetry data.
        Returns a dictionary with all telemetry fields from the drone.
        """
        with self._telemetry_lock:
            return self._telemetry.copy()
    
    def requestAllStates(self, verbose=False):
        """
        Get all aircraft states from telemetry.
        Note: You must call startTelemetryStream() first.
        """
        telemetry = self.getTelemetry()
        if verbose and telemetry:
            print("Telemetry:", json.dumps(telemetry, indent=2))
        return telemetry
    
    # Telemetry field accessors
    def getSpeed(self):
        """Get aircraft velocity (x, y, z)."""
        return self.getTelemetry().get("speed", {})
    
    def getHeading(self):
        """Get compass heading in degrees."""
        return self.getTelemetry().get("heading", 0.0)
    
    def getAttitude(self):
        """Get aircraft attitude (pitch, roll, yaw)."""
        return self.getTelemetry().get("attitude", {})
    
    def getLocation(self):
        """Get aircraft 3D location (latitude, longitude, altitude)."""
        return self.getTelemetry().get("location", {})
    
    def getGimbalAttitude(self):
        """Get gimbal attitude (pitch, roll, yaw)."""
        return self.getTelemetry().get("gimbalAttitude", {})
    
    def getGimbalJointAttitude(self):
        """Get gimbal joint attitude (pitch, roll, yaw)."""
        return self.getTelemetry().get("gimbalJointAttitude", {})
    
    def getZoomFocalLength(self):
        """Get camera zoom focal length."""
        return self.getTelemetry().get("zoomFl", -1)
    
    def getHybridFocalLength(self):
        """Get camera hybrid focal length."""
        return self.getTelemetry().get("hybridFl", -1)
    
    def getOpticalFocalLength(self):
        """Get camera optical focal length."""
        return self.getTelemetry().get("opticalFl", -1)
    
    def getZoomRatio(self):
        """Get camera zoom ratio."""
        return self.getTelemetry().get("zoomRatio", 1.0)
    
    def getBatteryLevel(self):
        """Get battery level percentage."""
        return self.getTelemetry().get("batteryLevel", -1)
    
    def getSatelliteCount(self):
        """Get GPS satellite count."""
        return self.getTelemetry().get("satelliteCount", -1)
    
    def getHomeLocation(self):
        """Get home location (latitude, longitude)."""
        return self.getTelemetry().get("homeLocation", {})
    
    def getDistanceToHome(self):
        """Get distance to home in meters."""
        return self.getTelemetry().get("distanceToHome", 0.0)

    def getLRFTarget(self):
        """Get the last LRF-locked target position (latitude, longitude, altitude)."""
        return self.getTelemetry().get("lrfTarget")

    def getWaypointSeq(self):
        """Id of the waypoint the streamed 'waypointReached' currently refers to.

        Mirrors DroneController._waypointSeq, incremented by the app for every
        requestSendGoToWPwithPID. Returns -1 if telemetry hasn't reported it yet.
        """
        return self.getTelemetry().get("waypointSeq", -1)

    def isWaypointReached(self, seq=None):
        """Check if a commanded waypoint has been reached."""
        telemetry = self.getTelemetry()
        reached = telemetry.get("waypointReached", False)
        if seq is None:
            return reached
        return reached and telemetry.get("waypointSeq", -1) == seq
    
    def isIntermediaryWaypointReached(self):
        """Check if an intermediary waypoint has been reached."""
        return self.getTelemetry().get("intermediaryWaypointReached", False)
    
    def getYawSeq(self):
        """Id of the gotoYaw command the streamed 'yawReached' refers to (-1 if unknown)."""
        return self.getTelemetry().get("yawSeq", -1)

    def isYawReached(self, seq=None):
        """Check if a commanded yaw has been reached.

        'yawReached' latches until the next command, so pass the seq returned by
        requestSendGotoYaw to require the streamed status belongs to that command.
        seq=None keeps the legacy (race-prone) raw-flag read.
        """
        telemetry = self.getTelemetry()
        reached = telemetry.get("yawReached", False)
        if seq is None:
            return reached
        return reached and telemetry.get("yawSeq", -1) == seq

    def getAltitudeSeq(self):
        """Id of the gotoAltitude command the streamed 'altitudeReached' refers to (-1 if unknown)."""
        return self.getTelemetry().get("altitudeSeq", -1)

    def isAltitudeReached(self, seq=None):
        """Check if a commanded altitude has been reached.

        Pass the seq returned by requestSendGotoAltitude to avoid the stale-latch race.
        seq=None keeps the legacy (race-prone) raw-flag read.
        """
        telemetry = self.getTelemetry()
        reached = telemetry.get("altitudeReached", False)
        if seq is None:
            return reached
        return reached and telemetry.get("altitudeSeq", -1) == seq
    
    def isCameraRecording(self):
        """Check if the camera is currently recording."""
        return self.getTelemetry().get("isRecording", False)
    
    def isHomeSet(self):
        """Check if the home location has been set."""
        return self.getTelemetry().get("homeSet", False)
    
    def getRemainingFlightTime(self):
        """Get remaining flight time in minutes."""
        return self.getTelemetry().get("remainingFlightTime", 0)
    
    def getTimeNeededToGoHome(self):
        """Get time needed to return home in seconds."""
        return self.getTelemetry().get("timeNeededToGoHome", 0)
    
    def getTimeNeededToLand(self):
        """Get time needed to land in seconds."""
        return self.getTelemetry().get("timeNeededToLand", 0)
    
    def getTotalTime(self):
        """Get total time needed (go home + land) in seconds."""
        return self.getTelemetry().get("totalTime", 0)
    
    def getMaxRadiusCanFlyAndGoHome(self):
        """Get maximum radius the drone can fly and still return home."""
        return self.getTelemetry().get("maxRadiusCanFlyAndGoHome", 0)
    
    def getRemainingCharge(self):
        """Get remaining battery charge percentage."""
        return self.getTelemetry().get("remainingCharge", 0)
    
    def getBatteryNeededToLand(self):
        """Get battery percentage needed to land."""
        return self.getTelemetry().get("batteryNeededToLand", 0)
    
    def getBatteryNeededToGoHome(self):
        """Get battery percentage needed to return home."""
        return self.getTelemetry().get("batteryNeededToGoHome", 0)
    
    def getSeriousLowBatteryThreshold(self):
        """Get serious low battery warning threshold percentage."""
        return self.getTelemetry().get("seriousLowBatteryThreshold", 0)
    
    def getLowBatteryThreshold(self):
        """Get low battery warning threshold percentage."""
        return self.getTelemetry().get("lowBatteryThreshold", 0)
    
    def getFlightMode(self):
        """Get the current flight mode (e.g., 'MANUAL', 'GPS', 'GO_HOME', etc.)."""
        return self.getTelemetry().get("flightMode", "UNKNOWN")

    def isManualOverrideActive(self):
        """Check if manual override is active (pilot took RC control).
        
        When True, autonomous HTTP commands are being rejected by the app.
        The pilot must deactivate manual override before autonomous commands work again.
        """
        return self.getTelemetry().get("isManualOverrideActive", False)

    # ==================== Commands (HTTP POST on port 8080) ====================

    def requestSend(self, endPoint, data, verbose=False):
        """Send a POST request to the drone."""
        if self.IP_RC == "":
            print(f"No IP_RC provided, returning empty string for request at {endPoint}")
            return ""
        try:
            response = requests.post(self.baseCommandUrl + endPoint, str(data), timeout=5)
            if verbose:
                print("EP : " + endPoint + "\t" + str(response.content, encoding="utf-8"))
            return response.content.decode('utf-8')
        except requests.exceptions.RequestException as e:
            print(f"Request error at {endPoint}: {e}")
            return ""

    def requestSendStick(self, leftX=0, leftY=0, rightX=0, rightY=0):
        """Send virtual stick commands. Values should be in [-1, 1]."""
        # Saturate values such that they are in [-1;1]
        s = 0.3
        leftX = max(-s, min(s, leftX))
        leftY = max(-s, min(s, leftY))
        rightX = max(-s, min(s, rightX))
        rightY = max(-s, min(s, rightY))
        rep = self.requestSend(
            EP_STICK, f"{leftX:.4f},{leftY:.4f},{rightX:.4f},{rightY:.4f}")
        return rep

    def requestSendGimbalPitch(self, pitch=0):
        """Set gimbal pitch angle."""
        return self.requestSend(EP_GIMBAL_SET_PITCH, f"0,{pitch},0")

    def requestSendGimbalYaw(self, yaw=0):
        """Set gimbal yaw angle."""
        return self.requestSend(EP_GIMBAL_SET_YAW, f"0,0,{yaw}")

    def requestSendZoomRatio(self, zoomRatio=1):
        """Set camera zoom ratio."""
        return self.requestSend(EP_ZOOM, zoomRatio)

    def requestSendTakeOff(self):
        """Command the drone to take off."""
        return self.requestSend(EP_TAKEOFF, "")

    def requestSendLand(self):
        """Command the drone to land."""
        return self.requestSend(EP_LAND, "")

    def requestSendRTH(self):
        """Command the drone to return to home.
        
        Note: This first aborts any active mission and disables virtual stick
        to prevent conflicts with RTH. Virtual stick mode can interfere with
        RTH causing erratic behavior.
        """
        # CRITICAL: Disable virtual stick before RTH to prevent conflicts
        self.requestAbortMission()
        return self.requestSend(EP_RTH, "")

    def requestSendGoToWP(self, latitude, longitude, altitude):
        """Navigate to a waypoint."""
        return self.requestSend(EP_GOTO_WP, f"{latitude},{longitude},{altitude}")

    def requestSendGoToWPwithPID(self, latitude, longitude, altitude, yaw, speed: float = 5.0):
        """Navigate to a waypoint with PID control.

        Args:
            latitude: Target latitude
            longitude: Target longitude
            altitude: Target altitude
            yaw: Target yaw angle
            speed: Max speed in m/s (default 5.0)

        Returns:
            int: the sequence id the app assigned to this request (parsed from the
                 "WAYPOINT_ACCEPTED seq=<n> ..." response). Pass it to
                 isWaypointReached(seq) to avoid the stale-latch race.
            None: if the command was rejected or the response had no seq.
        """
        response = self.requestSend(EP_GOTO_WP_PID, f"{latitude},{longitude},{altitude},{yaw},{speed}")
        return self._parseSeq(response)

    @staticmethod
    def _parseSeq(response):
        """Extract the integer seq from an '<X>_ACCEPTED seq=<n> ...' response, else None."""
        match = re.search(r"seq=(\d+)", str(response))
        return int(match.group(1)) if match else None
    
    def requestSendGoToWPwithPIDtuning(self, latitude, longitude, altitude, yaw, kp_pos, ki_pos, kd_pos, kp_yaw, ki_yaw, kd_yaw):
        """Navigate to a waypoint with custom PID tuning parameters."""
        return self.requestSend(EP_TUNING, f"{latitude},{longitude},{altitude},{yaw},{kp_pos},{ki_pos},{kd_pos},{kp_yaw},{ki_yaw},{kd_yaw}")

    def requestCapture(self):
        """Trigger ONE H20T shutter (no image download). Returns the capture descriptor.
        Returns:
            dict {"thermal": fn|None, "wide": fn|None, "zoom": fn|None} on success (fn is the
            on-camera filename, None if that lens was not stored), else False.

        Download any returned filename with downloadByName().
        """

        if self.IP_RC == "":
            print("No IP_RC provided, cannot capture image")
            return False
        # Trip the shutter. The bridge returns a JSON descriptor naming the on-camera filename
        # of each lens the H20T stored (no image yet).
        try:
            # Generous timeout: the very first capture after connect can be cold (the bridge builds
            # the full SD-card list once), so allow well past the server's internal resolution cap.
            response = requests.post(
                self.baseCommandUrl + EP_CAPTURE_THERMAL_IMAGE, data="", timeout=60)
        except requests.exceptions.RequestException as e:
            print(f"Error capturing image: {e}")
            return False
        try:
            info = response.json()
        except ValueError:
            print(f"Capture returned non-JSON: HTTP {response.status_code}, "
                  f"body={response.text[:200]!r}")
            return False
        if info.get("error") or not info.get("thermal"):
            print(f"Capture failed: {info}")
            return False
        return info

    def listMedia(self):
        """List every file on the camera's SD card (robust path — source of truth, not the
        bounded recent-capture cache).

        Returns:
            list of dicts {"name": str, "index": int, "size": int, "type": str} on success,
            else False.
        """
        if self.IP_RC == "":
            print("No IP_RC provided, cannot list media")
            return False
        try:
            response = requests.post(
                self.baseCommandUrl + EP_LIST_MEDIA, data="", timeout=30)
        except requests.exceptions.RequestException as e:
            print(f"Error listing media: {e}")
            return False
        try:
            info = response.json()
        except ValueError:
            print(f"listMedia returned non-JSON: HTTP {response.status_code}, "
                  f"body={response.text[:200]!r}")
            return False
        return info.get("files", [])

    def downloadByName(self, file_name, save_path=None, out_dir="."):
        """Download ANY file from the SD card by its on-camera filename. Works for any file the
        camera ever wrote, regardless of how many captures happened since — no dependence on the
        bounded recent-capture cache.

        Args:
            file_name: the on-camera filename (e.g. from listMedia() or a capture descriptor).
            save_path: full output path; defaults to out_dir/file_name.
            out_dir: directory used when save_path is not given (created if missing).

        Returns:
            the saved path, or None on failure.
        """
        if self.IP_RC == "":
            print("No IP_RC provided, cannot download image")
            return None
        if not file_name:
            print("Download error: no file_name")
            return None
        if save_path is None:
            os.makedirs(out_dir, exist_ok=True)
            save_path = os.path.join(out_dir, file_name)
        try:
            response = requests.post(
                self.baseCommandUrl + EP_DOWNLOAD_MEDIA_BY_NAME,
                data=file_name, timeout=120)
        except requests.exceptions.RequestException as e:
            print(f"{file_name}: download error: {e}")
            return None
        content_type = response.headers.get("Content-Type", "")
        if response.status_code != 200 or not content_type.startswith("image/"):
            print(f"{file_name}: download failed (HTTP {response.status_code}, "
                  f"Content-Type={content_type!r}, body={response.text[:200]!r})")
            return None
        with open(save_path, "wb") as f:
            f.write(response.content)
        print(f"{file_name} saved to: {save_path} ({len(response.content)} bytes)")
        return save_path

    def requestLRFMeasure(self):
        """Fire the H20T laser range finder once and return its reading."""
        response = self.requestSend(EP_LRF_MEASURE, "")
        if not response:
            return {"distance": None, "target": None, "state": None}
        try:
            print(response)
            return json.loads(response)
        except ValueError:
            print(f"LRF: could not parse response: {response!r}")
            return {"distance": None, "target": None, "state": None}  
        
    def requestSendNavigateTrajectory(self, waypoints, finalYaw):
        """
        Navigate through a series of waypoints.
        :param waypoints: A list of triples (latitude, longitude, altitude) for each waypoint.
        :param finalYaw: The final yaw angle at the last waypoint.
        :return: The response from the server.
        """
        self.requestSendEnableVirtualStick()
        if not waypoints:
            raise ValueError("No waypoints provided")

        # Build the message
        # All waypoints except the last: "lat,lon,alt"
        # Last waypoint: "lat,lon,alt,yaw"
        segments = []
        for i, (lat, lon, alt) in enumerate(waypoints):
            if i < len(waypoints) - 1:
                # Intermediary waypoint: lat,lon,alt
                segments.append(f"{lat},{lon},{alt}")
            else:
                # Last waypoint: lat,lon,alt,yaw
                segments.append(f"{lat},{lon},{alt},{finalYaw}")

        message = ";".join(segments)
        return self.requestSend(EP_GOTO_TRAJECTORY, message)
    
    def requestSendNavigateTrajectoryDJINative(self, waypoints, speed: float = 10.0):
        """
        Send waypoints to be executed using DJI's native waypoint mission system.
        :param waypoints: A list of triples (latitude, longitude, altitude) for each waypoint.
        :param speed: Flight speed in m/s (default 10.0)
        :return: The response from the server.
        """
        if not waypoints:
            raise ValueError("No waypoints provided")
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints for DJI native mission")

        # Build the message format: "speed;lat,lon,alt;lat,lon,alt;..."
        segments = [str(speed)]
        for lat, lon, alt in waypoints:
            segments.append(f"{lat},{lon},{alt}")

        message = ";".join(segments)
        return self.requestSend(EP_GOTO_TRAJECTORY_DJI_NATIVE, message)
    
    def requestAbortDJINativeMission(self):
        """Abort the current DJI native waypoint mission."""
        return self.requestSend(EP_ABORT_DJI_NATIVE_MISSION, "")

    def requestAbortMission(self):
        """Abort the current mission and disable virtual stick."""
        return self.requestSend(EP_ABORT_MISSION, "")

    def requestSendEnableVirtualStick(self):
        """Enable virtual stick control mode."""
        return self.requestSend(EP_ENABLE_VIRTUAL_STICK, "")

    def requestSendGotoYaw(self, yaw):
        """Rotate to a specific yaw angle.

        Returns the int seq the app assigned (for isYawReached(seq)), or None if rejected.
        """
        self.requestSendEnableVirtualStick()
        return self._parseSeq(self.requestSend(EP_GOTO_YAW, f"{yaw}"))

    def requestSendGotoAltitude(self, altitude):
        """Navigate to a specific altitude.

        Returns the int seq the app assigned (for isAltitudeReached(seq)), or None if rejected.
        """
        self.requestSendEnableVirtualStick()
        return self._parseSeq(self.requestSend(EP_GOTO_ALTITUDE, f"{altitude}"))

    def requestCameraStartRecording(self):
        """Start camera recording."""
        return self.requestSend(EP_CAMERA_START_RECORDING, "")

    def requestCameraStopRecording(self):
        """Stop camera recording."""
        return self.requestSend(EP_CAMERA_STOP_RECORDING, "")
    
    def requestSetRTHAltitude(self, altitude):
        """Set the return-to-home altitude in meters."""
        return self.requestSend(EP_SET_RTH_ALTITUDE, str(altitude))

    def requestDeactivateManualOverride(self):
        """Deactivate manual override latch so autonomous commands are accepted again.
        
        This should be called after the pilot has finished manual control
        and wants to allow autonomous commands to work again.
        """
        return self.requestSend(EP_DEACTIVATE_MANUAL_OVERRIDE, "")
    
    def requestSticks(self):
        """Deprecated: RC stick values are now available via getTelemetry()."""
        print("Warning: requestSticks() is deprecated. Use getTelemetry() instead.")
        return ""

    def requestWaypointStatus(self):
        """Deprecated: Use isWaypointReached() instead."""
        return str(self.isWaypointReached()).lower()

    def requestIntermediaryWaypointStatus(self):
        """Deprecated: Use isIntermediaryWaypointReached() instead."""
        return str(self.isIntermediaryWaypointReached()).lower()

    def requestYawStatus(self):
        """Deprecated: Use isYawReached() instead."""
        return str(self.isYawReached()).lower()

    def requestAltitudeStatus(self):
        """Deprecated: Use isAltitudeReached() instead."""
        return str(self.isAltitudeReached()).lower()

    def requestHomePosition(self):
        """Deprecated: Use getHomeLocation() instead."""
        return self.getHomeLocation()

    def requestCameraIsRecording(self):
        """Deprecated: Use isCameraRecording() instead."""
        return self.isCameraRecording()
    
    def requestDrop(self):
        """Drop the payload."""
        return self.requestSend(EP_PAYLOAD_DROP, "")

class DJIInterfaceLite:
    def __init__(self, IP_RC=""):
        self.IP_RC = IP_RC
        self.baseTelemUrl = f"http://{IP_RC}:8080"
        self.videoSource = f"rtsp://aaa:aaa@{self.IP_RC}:8554/streaming/live/1"
        self.imagePaths = []

    def getVideoSource(self):
        if self.IP_RC == "":
            return ""
        return self.videoSource

    def requestGet(self, endPoint, verbose=False):
        if self.IP_RC == "":
            return ""
        response = requests.get(self.baseTelemUrl + endPoint)
        if verbose:
            print("EP : " + endPoint + "\t" +
                  str(response.content, encoding="utf-8"))
        return response.content.decode('utf-8')

    def requestSend(self, endPoint, data, verbose=False):
        if self.IP_RC == "":
            print(
                f"No IP_RC provided, returning empty string for request at {endPoint}")
            return ""
        response = requests.post(self.baseTelemUrl + endPoint, str(data))
        if verbose:
            print("EP : " + endPoint + "\t" +
                  str(response.content, encoding="utf-8"))
        return response.content.decode('utf-8')

    def requestAllStates(self, verbose=False):
        response = self.requestGet(EP_ALL_STATES, verbose)
        response = response.replace("null", "None")
        try:
            # TODO: probably very unsafe!!!
            states = ast.literal_eval(response)
            states["timestamp"] = datetime.now().strftime(
                "%Y-%m-%d_%H-%M-%S.%f")

            return states
        except:
            return {}

    def requestSendStick(self, leftX=0, leftY=0, rightX=0, rightY=0):
        # Saturate values such that they are in [-1;1]
        s = 0.3
        leftX = max(-s, min(s, leftX))
        leftY = max(-s, min(s, leftY))
        rightX = max(-s, min(s, rightX))
        rightY = max(-s, min(s, rightY))
        rep = self.requestSend(
            EP_STICK, f"{leftX:.4f},{leftY:.4f},{rightX:.4f},{rightY:.4f}")
        return rep

    def requestSendGimbalPitch(self, pitch=0):
        return self.requestSend(EP_GIMBAL_SET_PITCH, f"0,{pitch},0")

    def requestSendGimbalYaw(self, yaw=0):
        return self.requestSend(EP_GIMBAL_SET_YAW, f"0,0,{yaw}")

    def requestSendZoomRatio(self, zoomRatio=1):
        return self.requestSend(EP_ZOOM, zoomRatio)

    def requestSendTakeOff(self):
        return self.requestSend(EP_TAKEOFF, "")

    def requestSendLand(self):
        return self.requestSend(EP_LAND, "")

    def requestSendRTH(self):
        return self.requestSend(EP_RTH, "")

    def requestSendGoToWP(self, latitude, longitude, altitude):
        return self.requestSend(EP_GOTO_WP, f"{latitude},{longitude},{altitude}")

    def requestSendGoToWPwithPID(self, latitude, longitude, altitude, yaw):
        return self.requestSend(EP_GOTO_WP_PID, f"{latitude},{longitude},{altitude},{yaw}")
    
    def requestSendGoToWPwithPIDtuning(self, latitude, longitude, altitude, yaw, kp_pos, ki_pos, kd_pos, kp_yaw, ki_yaw, kd_yaw):
        return self.requestSend(EP_TUNING, f"{latitude},{longitude},{altitude},{yaw},{kp_pos},{ki_pos},{kd_pos},{kp_yaw},{ki_yaw},{kd_yaw}")

    def requestSendNavigateTrajectory(self, waypoints, finalYaw):
        """
        :param waypoints: A list of triples (latitude, longitude, altitude) for each waypoint.
        :param finalYaw: The final yaw angle at the last waypoint.
        :return: The response from the server.
        """
        self.requestSendEnableVirtualStick()
        if not waypoints:
            raise ValueError("No waypoints provided")

        # Build the message
        # All waypoints except the last: "lat,lon,alt"
        # Last waypoint: "lat,lon,alt,yaw"
        segments = []
        for i, (lat, lon, alt) in enumerate(waypoints):
            if i < len(waypoints) - 1:
                # Intermediary waypoint: lat,lon,alt
                segments.append(f"{lat},{lon},{alt}")
            else:
                # Last waypoint: lat,lon,alt,yaw
                segments.append(f"{lat},{lon},{alt},{finalYaw}")

        message = ";".join(segments)
        return self.requestSend(EP_GOTO_TRAJECTORY, message)
    
    def requestDrop(self):
        # TODO: Not safe now, need to get the payload information and check the payload availability maybe
        return self.requestSend(EP_PAYLOAD_DROP, "")
    
    def requestSendNavigateTrajectoryDJINative(self, waypoints):
        """
        Send waypoints to be executed using DJI's native waypoint mission system.
        :param waypoints: A list of triples (latitude, longitude, altitude) for each waypoint.
        :return: The response from the server.
        """
        if not waypoints:
            raise ValueError("No waypoints provided")

        # Build the message format: "lat,lon,alt; lat,lon,alt; ..."
        segments = []
        for lat, lon, alt in waypoints:
            segments.append(f"{lat},{lon},{alt}")

        message = ";".join(segments)
        return self.requestSend(EP_GOTO_TRAJECTORY_DJI_NATIVE, message)

    def requestCaptureThermalImage(self, save_path=None):
        """
        Requests the drone to capture a thermal image and optionally saves it.
        Args:
            save_path (str, optional): Path where to save the image. If None, generates a timestamped filename.
        Returns:
            str: Path to the saved image if successful, None otherwise.
        """
        if self.IP_RC == "":
            print("No IP_RC provided, cannot capture thermal image")
            return None
            
        # Generate default save path if none provided
        if save_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = f"thermal_image_{timestamp}.jpg"
            
        try:
            response = requests.post(self.baseTelemUrl + EP_CAPTURE_THERMAL_IMAGE,
                                  data="",
                                  timeout=20)

            if response.headers.get('Content-Type', '').startswith('image/'):
                with open(save_path, 'wb') as f:
                    f.write(response.content)
                print(f"Thermal image saved to: {save_path}")
                self.imagePaths.append(save_path)
                return SAVE_SUCCESS
            else:
                print(f"Error: Received non-image response: {response.text}")
                return SAVE_FAILURE

        except Exception as e:
            print(f"Error capturing thermal image: {str(e)}")
            return CAP_FAILURE
    
    def requestAbortDJINativeMission(self):
        return self.requestSend(EP_ABORT_DJI_NATIVE_MISSION, "")

    def requestSticks(self):
        return self.requestGet(EP_STICK_VALUES, True)

    def requestAbortMission(self):
        return self.requestSend(EP_ABORT_MISSION, "")

    def requestWaypointStatus(self):
        return self.requestGet(EP_WP_REACHED)

    def requestIntermediaryWaypointStatus(self):
        return self.requestGet(EP_INTERMEDIARY_WP_REACHED)

    def requestSendEnableVirtualStick(self):
        return self.requestSend(EP_ENABLE_VIRTUAL_STICK, "")

    def requestYawStatus(self):
        return self.requestGet(EP_YAW_REACHED)

    def requestSendGotoYaw(self, yaw):
        return self.requestSend(EP_GOTO_YAW, f"{yaw}")

    def requestSendGotoAltitude(self, altitude):
        return self.requestSend(EP_GOTO_ALTITUDE, f"{altitude}")

    def requestAltitudeStatus(self):
        return self.requestGet(EP_ALTITUDE_REACHED)

    def requestHomePosition(self):
        response = self.requestGet(EP_HOME_LOCATION, False)
        try:
            # TODO: probably very unsafe!!!
            home = ast.literal_eval(response)
            return home
        except:
            return {}

    def requestCameraStartRecording(self):
        return self.requestSend(EP_CAMERA_START_RECORDING, "")

    def requestCameraStopRecording(self):
        return self.requestSend(EP_CAMERA_STOP_RECORDING, "")

    def requestCameraIsRecording(self):
        return self.requestGet(EP_CAMERA_IS_RECORDING) == "true"

if __name__ == '__main__':
    import time
    import sys
    
    IP_RC = "172.20.10.12"  # REPLACE WITH YOUR RC IP
    
    if len(sys.argv) > 1:
        IP_RC = sys.argv[1]

    print(f"Connecting to {IP_RC}...")
    dji = DJIInterface(IP_RC)

    # Start telemetry stream (TCP socket on port 8081)
    print("Starting telemetry stream...")
    dji.startTelemetryStream()
    
    # Wait for initial connection
    time.sleep(1)
    
    print("\n" + "="*60)
    print("TCP Telemetry Socket Test - Press Ctrl+C to stop")
    print("="*60 + "\n")
    
    try:
        while True:
            telemetry = dji.getTelemetry()
            
            if telemetry:
                # Clear screen effect by printing separator
                print("-" * 60)
                print(f"[{telemetry.get('timestamp', 'N/A')}]")
                print(f"  Battery:     {dji.getBatteryLevel()}%")
                print(f"  Satellites:  {dji.getSatelliteCount()}")
                print(f"  Heading:     {dji.getHeading():.1f}°")
                print(f"  Location:    {dji.getLocation()}")
                print(f"  Altitude:    {dji.getLocation().get('altitude', 'N/A')} m")
                print(f"  Speed:       {dji.getSpeed()}")
                print(f"  Attitude:    {dji.getAttitude()}")
                print(f"  Gimbal:      {dji.getGimbalAttitude()}")
                print(f"  Home Set:    {dji.isHomeSet()}")
                print(f"  Home Loc:    {dji.getHomeLocation()}")
                print(f"  Dist Home:   {dji.getDistanceToHome():.1f} m")
                print(f"  Recording:   {dji.isCameraRecording()}")
                print(f"  WP Reached:  {dji.isWaypointReached()}")
                print(f"  Yaw Reached: {dji.isYawReached()}")
                print(f"  Alt Reached: {dji.isAltitudeReached()}")
                print(f"  Flight Time: {dji.getRemainingFlightTime()} s remaining")
                print(f"  Total Time:  {dji.getTotalTime()} s")
                print(f"  Time to RTH: {dji.getTimeNeededToGoHome()} s")
                print(f"  Time to Land:{dji.getTimeNeededToLand()} s")
                print(f"  Max Radius:  {dji.getMaxRadiusCanFlyAndGoHome()} m")
                print(f"  --- Battery Thresholds ---")
                print(f"  Remaining:   {dji.getRemainingCharge()}%")
                print(f"  Need Land:   {dji.getBatteryNeededToLand()}%")
                print(f"  Need RTH:    {dji.getBatteryNeededToGoHome()}%")
                print(f"  Low Batt:    {dji.getLowBatteryThreshold()}%")
                print(f"  Serious Low: {dji.getSeriousLowBatteryThreshold()}%")
                print(f"  Flight Mode: {dji.getFlightMode()}")
                print(f"  Manual Override: {dji.isManualOverrideActive()}")
            else:
                print("Waiting for telemetry data...")
            
            time.sleep(0.1)  # Update every 500ms
            
    except KeyboardInterrupt:
        print("\n\nStopping telemetry stream...")
        dji.stopTelemetryStream()
        print("Done.")

