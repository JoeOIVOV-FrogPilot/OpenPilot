#!/usr/bin/env python3
import json
import math
import re
import unicodedata

import requests

from collections import deque
from datetime import datetime, timedelta, UTC

from cereal import messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params

from openpilot.frogpilot.common.frogpilot_utilities import calculate_distance_to_point, is_url_pingable

OVERPASS_ENDPOINT = "https://overpass-api.de/api/interpreter"

MAX_SPEED_LIMITS = 1_000_000
VETTING_INTERVAL_DAYS = 7

BBOX_PAD = 0.001
BEARING_TOLERANCE = 40
GRID_SIZE = 0.01
NAME_MATCH_BONUS = 8
SNAP_DISTANCE = 30

DRIVABLE_HIGHWAYS = "motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street|service"


def bearing_difference(first, second):
  difference = abs(first - second) % 180
  return min(difference, 180 - difference)


class Coordinate:
  def __init__(self, latitude, longitude):
    self.latitude = latitude
    self.longitude = longitude

  def __add__(self, other):
    return Coordinate(self.latitude + other.latitude, self.longitude + other.longitude)

  def __mul__(self, scalar):
    return Coordinate(self.latitude * scalar, self.longitude * scalar)

  def __sub__(self, other):
    return Coordinate(self.latitude - other.latitude, self.longitude - other.longitude)

  def distance_to(self, other):
    return calculate_distance_to_point(self.latitude, self.longitude, other.latitude, other.longitude)

  def dot(self, other):
    return self.latitude * other.latitude + self.longitude * other.longitude


def has_stacked_conflict(latitude, longitude, snapped, ways):
  snapped_name = normalize_name(snapped.get("tags", {}).get("name") or snapped.get("tags", {}).get("ref") or "")
  if not snapped_name:
    return False

  snapped_layer = snapped.get("tags", {}).get("layer") or "0"
  point = Coordinate(latitude, longitude)
  for way in ways:
    tags = way.get("tags", {})

    if way.get("id") == snapped.get("id") or (tags.get("layer") or "0") == snapped_layer:
      continue

    if normalize_name(tags.get("name") or tags.get("ref") or "") != snapped_name:
      continue

    coordinates = (way.get("geometry") or {}).get("coordinates", [])
    if not near_point(latitude, longitude, coordinates, SNAP_DISTANCE / 111000 + 0.0005):
      continue
    for start, end in zip(coordinates, coordinates[1:]):
      try:
        if minimum_distance(Coordinate(start[1], start[0]), Coordinate(end[1], end[0]), point) <= SNAP_DISTANCE:
          return True
      except (IndexError, TypeError):
        continue
  return False


def heading_difference(bearing, start, end, oneway):
  heading = segment_bearing(start, end)
  if isinstance(oneway, str):
    oneway = oneway.strip().lower()
  if oneway == "-1":
    heading = (heading + 180) % 360
  if oneway in ("-1", "1", "true", "yes"):
    difference = abs(bearing - heading) % 360
    return min(difference, 360 - difference)
  return bearing_difference(bearing, heading)


def load_dict(raw):
  try:
    value = json.loads(raw or "{}")
  except (TypeError, ValueError):
    return {}
  return value if isinstance(value, dict) else {}


def load_records(raw):
  try:
    records = json.loads(raw or "[]")
  except (TypeError, ValueError):
    return []
  return records if isinstance(records, list) else []


def minimum_distance(start, end, point):
  if start.distance_to(end) < 0.01:
    return start.distance_to(point)

  segment = end - start
  offset = point - start
  t = max(0.0, min(1.0, offset.dot(segment) / segment.dot(segment)))
  return (start + segment * t).distance_to(point)


def near_point(latitude, longitude, coordinates, pad):
  valid = [coordinate for coordinate in coordinates if isinstance(coordinate, (list, tuple)) and len(coordinate) >= 2]
  valid = [coordinate for coordinate in valid if valid_coordinate(coordinate[0]) and valid_coordinate(coordinate[1])]
  if not valid:
    return False
  longitudes = [coordinate[0] for coordinate in valid]
  latitudes = [coordinate[1] for coordinate in valid]
  return min(longitudes) - pad <= longitude <= max(longitudes) + pad and min(latitudes) - pad <= latitude <= max(latitudes) + pad


def normalize_name(name):
  if not isinstance(name, str):
    return ""

  name = unicodedata.normalize("NFKC", name).casefold().replace("-", " ").replace("/", " ")
  name = "".join(character for character in name if character not in ".,'")

  words = name.split()
  while words and words[0] == "the":
    words = words[1:]
  while words and words[-1] == "the":
    words = words[:-1]

  return " ".join(words)


def parse_float(value):
  try:
    return float(value)
  except (TypeError, ValueError):
    return 0.0


def parse_osm_maxspeed(maxspeed):
  if maxspeed is None:
    return 0

  if not isinstance(maxspeed, str):
    return None

  value = next((part.strip().lower() for part in maxspeed.split(";") if part.strip()), "")
  if not value:
    return 0

  match = re.match(r"^(\d+(?:\.\d+)?)\s*(mph|km/?h|kph)?$", value)
  if not match:
    return None

  speed = float(match.group(1))
  if not math.isfinite(speed):
    return None
  if match.group(2) == "mph":
    return speed * CV.MPH_TO_MS
  return speed * CV.KPH_TO_MS


def parse_timestamp(value):
  try:
    parsed = datetime.fromisoformat(value)
  except (TypeError, ValueError):
    return None
  return parsed if parsed.tzinfo is not None else parsed.replace(tzinfo=UTC)


def segment_bearing(start, end):
  longitude1, latitude1, longitude2, latitude2 = map(math.radians, (start[0], start[1], end[0], end[1]))
  delta_longitude = longitude2 - longitude1
  bearing_y = math.sin(delta_longitude) * math.cos(latitude2)
  bearing_x = math.cos(latitude1) * math.sin(latitude2) - math.sin(latitude1) * math.cos(latitude2) * math.cos(delta_longitude)
  bearing = math.degrees(math.atan2(bearing_y, bearing_x)) % 360
  return bearing if bearing < 360 else 0.0


def snap_to_way(latitude, longitude, ways, bearing, road_name):
  point = Coordinate(latitude, longitude)
  target_name = normalize_name(road_name)

  pad = SNAP_DISTANCE / 111000 + 0.0005

  closest_score = SNAP_DISTANCE
  closest_way = None

  for way in ways:
    coordinates = (way.get("geometry") or {}).get("coordinates", [])
    if not near_point(latitude, longitude, coordinates, pad):
      continue

    tags = way.get("tags", {})
    oneway = tags.get("oneway")
    name_match = bool(target_name) and target_name in (normalize_name(tags.get("name", "")), normalize_name(tags.get("ref", "")))

    for start, end in zip(coordinates, coordinates[1:]):
      try:
        if bearing is not None and heading_difference(bearing, start, end, oneway) > BEARING_TOLERANCE:
          continue

        distance = minimum_distance(Coordinate(start[1], start[0]), Coordinate(end[1], end[0]), point)
      except (IndexError, TypeError):
        continue

      if distance > SNAP_DISTANCE:
        continue

      score = distance - (NAME_MATCH_BONUS if name_match else 0)
      if score < closest_score:
        closest_score = score
        closest_way = way

  return closest_way


def valid_coordinate(value):
  return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def valid_entry(entry):
  if not isinstance(entry, dict) or not valid_coordinate(entry.get("latitude")) or not valid_coordinate(entry.get("longitude")):
    return False
  try:
    hash(entry.get("segment_id"))
  except TypeError:
    return False
  return True


def valid_way(way):
  if not isinstance(way, dict) or way.get("id") is None or not isinstance(way.get("tags", {}), dict):
    return False
  try:
    hash(way.get("id"))
  except TypeError:
    return False
  geometry = way.get("geometry")
  return isinstance(geometry, dict) and isinstance(geometry.get("coordinates"), list)


def vetting_keep(entry, osm_speed_limit):
  if entry.get("incorrect_limit"):
    return osm_speed_limit is None or osm_speed_limit < 1 or abs(osm_speed_limit - parse_float(entry.get("speed_limit"))) >= 1
  return osm_speed_limit is None or osm_speed_limit < 1


class SpeedLimitFiller:
  def __init__(self):
    self.session = requests.Session()

    self.params = Params(return_defaults=True)
    self.params_memory = Params(memory=True)

    self.started_previously = False

    self.logged_position = None

    self.speed_limits = deque(maxlen=MAX_SPEED_LIMITS)

    self.sm = messaging.SubMaster(["deviceState", "frogpilotCarState", "frogpilotPlan"])

  def filter_speed_limits(self, speed_limits):
    if not is_url_pingable(OVERPASS_ENDPOINT):
      return

    existing = load_records(self.params.get("SpeedLimitsFiltered"))
    if not speed_limits and not existing:
      return

    way_cache = {}

    def ways_for(latitude, longitude):
      self.sm.update(0)

      if self.sm["deviceState"].started:
        return None

      cell = (round(latitude / GRID_SIZE), round(longitude / GRID_SIZE))
      if cell not in way_cache:
        center_latitude, center_longitude = cell[0] * GRID_SIZE, cell[1] * GRID_SIZE
        bounding_box = (center_latitude - GRID_SIZE / 2 - BBOX_PAD, center_longitude - GRID_SIZE / 2 - BBOX_PAD,
                        center_latitude + GRID_SIZE / 2 + BBOX_PAD, center_longitude + GRID_SIZE / 2 + BBOX_PAD)
        way_cache[cell] = self.query_overpass(bounding_box)
      return way_cache[cell]

    now = datetime.now(UTC)

    filtered = deque(maxlen=MAX_SPEED_LIMITS)

    confirmed_segments = set()
    unverified_segments = set()

    for entry in existing:
      if not valid_entry(entry):
        continue

      last_vetted = parse_timestamp(entry.get("last_vetted"))
      if last_vetted is not None and timedelta(0) <= now - last_vetted < timedelta(days=VETTING_INTERVAL_DAYS):
        filtered.append(entry)
        confirmed_segments.add(entry.get("segment_id"))
        continue

      ways = ways_for(entry["latitude"], entry["longitude"])
      if not ways:
        filtered.append(entry)
        unverified_segments.add(entry.get("segment_id"))
        continue

      if vetting_keep(entry, self.segment_speed_limit(ways, entry.get("segment_id"))):
        entry["last_vetted"] = now.isoformat()
        filtered.append(entry)
        confirmed_segments.add(entry.get("segment_id"))

    deferred = []
    for entry in speed_limits:
      if not valid_entry(entry):
        continue

      ways = ways_for(entry["latitude"], entry["longitude"])
      if not ways:
        deferred.append(entry)
        continue

      way = self.nearest_way(entry["latitude"], entry["longitude"], ways, entry.get("bearing"), entry.get("road_name", ""))
      if way is None:
        deferred.append(entry)
        continue

      tags = way.get("tags", {})
      road_name = normalize_name(entry.get("road_name", ""))
      osm_name = normalize_name(tags.get("name", ""))
      osm_ref = normalize_name(tags.get("ref", ""))

      if (osm_name or osm_ref) and road_name not in (osm_name, osm_ref):
        deferred.append(entry)
        continue

      if has_stacked_conflict(entry["latitude"], entry["longitude"], way, ways):
        continue

      if way.get("id") in confirmed_segments:
        continue

      if way.get("id") in unverified_segments:
        deferred.append(entry)
        continue

      osm_speed_limit = parse_osm_maxspeed(tags.get("maxspeed"))
      if osm_speed_limit is None:
        deferred.append(entry)
        continue

      if entry.get("incorrect_limit"):
        if osm_speed_limit >= 1 and abs(osm_speed_limit - parse_float(entry.get("speed_limit"))) >= 1:
          filtered.append(self.confirmed_record(entry, way, now))
          confirmed_segments.add(way.get("id"))
      elif osm_speed_limit < 1:
        if any(tags.get(directional) for directional in ("maxspeed_forward", "maxspeed_backward", "maxspeed_conditional")):
          continue
        filtered.append(self.confirmed_record(entry, way, now))
        confirmed_segments.add(way.get("id"))

    self.params.put("SpeedLimitsFiltered", json.dumps(list(filtered)))
    self.params.put("SpeedLimits", json.dumps(deferred))

  @staticmethod
  def confirmed_record(entry, way, now):
    return {
      "incorrect_limit": entry.get("incorrect_limit", False),
      "last_vetted": now.isoformat(),
      "latitude": entry["latitude"],
      "longitude": entry["longitude"],
      "road_name": entry.get("road_name", ""),
      "segment_id": way.get("id"),
      "source": entry.get("source", ""),
      "speed_limit": entry.get("speed_limit", 0),
    }

  @staticmethod
  def segment_speed_limit(ways, segment_id):
    for way in ways:
      if way.get("id") == segment_id:
        return parse_osm_maxspeed(way.get("tags", {}).get("maxspeed"))
    return None

  def log_speed_limit(self):
    sources = (
      ("Dashboard", self.sm["frogpilotCarState"].dashboardSpeedLimit),
      ("Mapbox", self.sm["frogpilotPlan"].slcMapboxSpeedLimit),
    )
    source, reference_speed_limit = next(((name, limit) for name, limit in sources if limit >= 1), ("", 0))
    if reference_speed_limit < 1:
      return

    gps_position = load_dict(self.params_memory.get("LastGPSPosition"))
    latitude, longitude = gps_position.get("latitude"), gps_position.get("longitude")
    if not valid_coordinate(latitude) or not valid_coordinate(longitude):
      return

    bearing = gps_position.get("bearing")
    if self.logged_position and calculate_distance_to_point(*self.logged_position, latitude, longitude) < 1:
      return

    road_name = self.params_memory.get("RoadName") or ""
    if not road_name:
      return

    map_speed_limit = parse_float(self.params_memory.get("MapSpeedLimit"))
    if map_speed_limit >= 1:
      if abs(map_speed_limit - reference_speed_limit) < 1:
        return
      incorrect_limit = True
    else:
      incorrect_limit = False

    self.speed_limits.append(
      {
        "bearing": bearing,
        "incorrect_limit": incorrect_limit,
        "latitude": latitude,
        "longitude": longitude,
        "road_name": road_name,
        "source": source,
        "speed_limit": reference_speed_limit,
      }
    )
    self.logged_position = (latitude, longitude)

  def nearest_way(self, latitude, longitude, ways, bearing, road_name):
    way = snap_to_way(latitude, longitude, ways, bearing, road_name)
    if way is None and bearing is not None:
      way = snap_to_way(latitude, longitude, ways, None, road_name)
    return way

  def query_overpass(self, bounding_box):
    south, west, north, east = bounding_box
    query = f'[out:json][timeout:10];way["highway"~"^({DRIVABLE_HIGHWAYS})(_link)?$"]({south},{west},{north},{east});convert way ::id=id(),::geom=geom(),name=t["name"],ref=t["ref"],maxspeed=t["maxspeed"],maxspeed_forward=t["maxspeed:forward"],maxspeed_backward=t["maxspeed:backward"],maxspeed_conditional=t["maxspeed:conditional"],oneway=t["oneway"],layer=t["layer"];out geom;'

    try:
      response = self.session.get(OVERPASS_ENDPOINT, params={"data": query}, headers={"User-Agent": "FrogPilot SpeedLimitFiller"}, timeout=10)
      response.raise_for_status()
      elements = response.json()["elements"]
    except (requests.RequestException, ValueError, KeyError) as error:
      print(f"Overpass query failed: {error}")
      return None

    if not isinstance(elements, list):
      return None
    return [way for way in elements if valid_way(way)]

  def update(self):
    self.sm.update()

    started = self.sm["deviceState"].started

    if started and not self.started_previously:
      self.speed_limits = deque(maxlen=MAX_SPEED_LIMITS)

      self.logged_position = None
    elif not started and self.started_previously:
      merged = deque(load_records(self.params.get("SpeedLimits")), maxlen=MAX_SPEED_LIMITS)
      if self.speed_limits:
        merged.extend(self.speed_limits)
        self.params.put("SpeedLimits", json.dumps(list(merged)))

      self.filter_speed_limits(merged)
    elif started:
      self.log_speed_limit()

    self.started_previously = started


def main():
  speed_limit_filler = SpeedLimitFiller()

  while True:
    speed_limit_filler.update()


if __name__ == "__main__":
  main()
