#!/usr/bin/env python3
import bz2
import errno
import json
import os
import time

import capnp
import requests

import openpilot.system.sentry as sentry

from cereal import log, messaging
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr

from openpilot.frogpilot.common.frogpilot_utilities import get_frogpilot_api_info, is_url_pingable
from openpilot.frogpilot.common.frogpilot_variables import FROGPILOT_API

ERROR_BACKOFF = 60

EXPECTED_MISSING_FILE_ERRNOS = {errno.ENOENT, errno.ENOTDIR}

TELEMETRY_ENDPOINT = f"{FROGPILOT_API}/telemetry"
TELEMETRY_XATTR = "user.frogpilot_telemetry"

REDACTED_VIN = "VIN_REDACTED"

TELEMETRY_SERVICES = {
  # Raw vehicle signal discovery. Diagnostic/identifier frames are filtered below
  "can",
  "sendcan",

  # Car/control state for lateral and longitudinal tuning
  "carState",
  "carControl",
  "carOutput",
  "controlsState",
  "lateralPlanDEPRECATED",
  "longitudinalPlan",
  "onroadEvents",
  "qRoadEncodeIdx",
  "roadCameraState",
  "roadEncodeIdx",
  "uiPlan",
  "wideRoadCameraState",
  "wideRoadEncodeIdx",

  # Vehicle dynamics, calibration, and learned parameters
  "accelerometer",
  "accelerometer2",
  "cameraOdometry",
  "gyroscope",
  "gyroscope2",
  "liveCalibration",
  "liveDelay",
  "liveLocationKalman",
  "liveParameters",
  "livePose",
  "liveTorqueParameters",

  # Model/radar/traffic behavior
  "drivingModelData",
  "liveTracks",
  "modelV2",
  "navModel",
  "radarState",

  # Platform context. VIN and raw firmware payloads are redacted below
  "carParams",
  "frogpilotCarParams",
  "pandaStates",

  # FrogPilot-specific control and feature state
  "frogpilotCarControl",
  "frogpilotCarState",
  "frogpilotModelV2",
  "frogpilotOnroadEvents",
  "frogpilotPlan",
  "frogpilotRadarState",
}

KNOWN_VIN_DIAGNOSTIC_ADDRESSES = frozenset((0x760, 0x768, 0x7C6, 0x7CE))
STANDARD_DIAGNOSTIC_ADDRESSES = frozenset(range(0x7DF, 0x7F0))

VIN_DIAGNOSTIC_PATTERNS = (
  b"\x09\x02",       # OBD VIN request
  b"\x49\x02\x01",   # OBD VIN response
  b"\x22\xf1\x90",   # UDS VIN request
  b"\x62\xf1\x90",   # UDS VIN response
)


def clear_filter_state(filter_state):
  filter_state.valid = False

  filter_state.value = []
  filter_state.std = []


def clear_measurement(measurement):
  measurement.valid = False

  measurement.value = []
  measurement.std = []


def clear_xyz_measurement(measurement):
  measurement.valid = False

  measurement.x = 0
  measurement.y = 0
  measurement.z = 0
  measurement.xStd = 0
  measurement.yStd = 0
  measurement.zStd = 0


def is_extended_diagnostic_address(address):
  return 0x18DA0000 <= address <= 0x18DBFFFF


def anonymize_segment_name(segment):
  segment_number = segment.rsplit("--", 1)[-1]
  return f"segment-{segment_number}" if segment_number.isdigit() else "segment"


def is_sensitive_can_frame(can_frame):
  address = can_frame.address
  data = bytes(can_frame.dat)

  if (
    address in STANDARD_DIAGNOSTIC_ADDRESSES or
    address in KNOWN_VIN_DIAGNOSTIC_ADDRESSES or
    is_extended_diagnostic_address(address)
  ):
    return True

  return any(pattern in data for pattern in VIN_DIAGNOSTIC_PATTERNS)


def sanitize_can_messages(messages):
  return [
    {
      "address": can_frame.address,
      "dat": bytes(can_frame.dat),
      "src": can_frame.src,
    }
    for can_frame in messages
    if not is_sensitive_can_frame(can_frame)
  ]


def sanitize_car_control(car_control):
  car_control.orientationNED = []


def sanitize_car_params(car_params):
  car_params.carVin = REDACTED_VIN


def sanitize_frame_data(frame_data):
  frame_data.image = b""


def sanitize_event(event, service):
  if service in ("can", "sendcan"):
    sanitized_messages = sanitize_can_messages(getattr(event, service))
    setattr(event, service, sanitized_messages)
    return bool(sanitized_messages)

  if service == "carParams":
    sanitize_car_params(event.carParams)
  elif service in ("roadCameraState", "wideRoadCameraState"):
    sanitize_frame_data(getattr(event, service))
  elif service == "liveLocationKalman":
    sanitize_live_location_kalman(event.liveLocationKalman)
  elif service == "livePose":
    sanitize_live_pose(event.livePose)
  elif service == "carControl":
    sanitize_car_control(event.carControl)

  return True


def sanitize_live_location_kalman(live_location_kalman):
  clear_measurement(live_location_kalman.positionECEF)
  clear_measurement(live_location_kalman.positionGeodetic)
  clear_measurement(live_location_kalman.velocityECEF)
  clear_measurement(live_location_kalman.velocityNED)
  clear_measurement(live_location_kalman.orientationECEF)
  clear_measurement(live_location_kalman.calibratedOrientationECEF)
  clear_measurement(live_location_kalman.orientationNED)
  clear_measurement(live_location_kalman.calibratedOrientationNED)
  clear_measurement(live_location_kalman.filterState)

  live_location_kalman.gpsWeek = 0
  live_location_kalman.gpsTimeOfWeek = 0
  live_location_kalman.unixTimestampMillis = 0
  live_location_kalman.timeToFirstFix = 0


def sanitize_live_pose(live_pose):
  clear_xyz_measurement(live_pose.orientationNED)
  clear_filter_state(live_pose.debugFilterState)


def strip_log(raw_log):
  payload = bz2.decompress(raw_log) if raw_log.startswith(b"BZh") else raw_log

  scrubbed_messages = []
  for event in log.Event.read_multiple_bytes(payload):
    try:
      service = event.which()
    except capnp.lib.capnp.KjException:
      continue

    if service not in TELEMETRY_SERVICES:
      continue

    scrubbed_event = event.as_builder()
    if sanitize_event(scrubbed_event, service):
      scrubbed_messages.append(scrubbed_event.to_bytes())

  return bz2.compress(b"".join(scrubbed_messages))


class FrogPilotTelemetry:
  def __init__(self):
    self.session = requests.Session()

    self.started_previously = False
    self.pending = True

    self.sm = messaging.SubMaster(["deviceState"])

  def process_next_segment(self):
    roots = list(dict.fromkeys((Paths.log_root(), Paths.log_root(HD=True), Paths.log_root(konik=True))))
    for root in roots:
      try:
        segments = sorted(os.listdir(root))
      except OSError as error:
        if error.errno not in EXPECTED_MISSING_FILE_ERRNOS:
          sentry.capture_exception(error, crash_log=False)
        continue

      for segment in segments:
        rlog_path = os.path.join(root, segment, "rlog")

        try:
          if not os.path.isfile(rlog_path) or os.path.isfile(rlog_path + ".lock"):
            continue

          if getxattr(rlog_path, TELEMETRY_XATTR) == b"1":
            continue

          with open(rlog_path, "rb") as log_file:
            scrubbed = strip_log(log_file.read())

          if self.upload(segment, scrubbed):
            setxattr(rlog_path, TELEMETRY_XATTR, b"1")

          return True
        except OSError as error:
          if error.errno not in EXPECTED_MISSING_FILE_ERRNOS:
            sentry.capture_exception(error, crash_log=False)
          continue

    return False

  def update(self):
    self.sm.update()

    started = self.sm["deviceState"].started

    if not started and self.started_previously:
      self.pending = True

    self.started_previously = started

    if not self.pending:
      return

    if started:
      return

    if self.sm["deviceState"].networkType not in (log.DeviceState.NetworkType.ethernet, log.DeviceState.NetworkType.wifi):
      return

    if not self.process_next_segment():
      self.pending = False

  def upload(self, segment, payload):
    api_info = get_frogpilot_api_info()
    if not api_info.api_token or "frogai" not in api_info.build_metadata["openpilot"]["git_origin"].lower() or not is_url_pingable(FROGPILOT_API):
      return False

    anonymized_segment = anonymize_segment_name(segment)

    try:
      response = self.session.post(
        TELEMETRY_ENDPOINT,
        data={
          "api_token": api_info.api_token,
          "build_metadata": json.dumps(api_info.build_metadata),
          "device": api_info.device_type,
          "os_version": api_info.os_version or "",
          "segment": anonymized_segment,
        },
        files={"log": (f"{anonymized_segment}.bz2", payload, "application/octet-stream")},
        headers={"User-Agent": "frogpilot-api/1.0"},
        timeout=30,
      )
      response.raise_for_status()
      return True
    except requests.RequestException as error:
      print(f"Telemetry upload failed for {segment}: {error}")
      sentry.capture_exception(error, crash_log=False)
      return False


def main():
  frogpilot_telemetry = FrogPilotTelemetry()

  while True:
    try:
      frogpilot_telemetry.update()
    except Exception as error:
      sentry.capture_exception(error)
      time.sleep(ERROR_BACKOFF)


if __name__ == "__main__":
  main()
