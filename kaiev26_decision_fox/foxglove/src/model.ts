import { MessageEvent, Time } from "@foxglove/extension";

import {
  CameraFrame,
  EgoPose,
  Histories,
  MonitorModel,
  Point2,
  RosObject,
  TimedValue,
  ZonePath,
} from "./types";

export const TOPICS = {
  behavior: "/planning/behavior_decision",
  route: "/planning/route_context",
  routeReady: "/planning/route_ready",
  readiness: "/planning/readiness_status",
  scene: "/planning/scene_summary",
  scenario: "/decision/scenario",
  targetPath: "/planning/target_path",
  targetSpeed: "/planning/target_speed",
  rawCommand: "/decision/raw_command",
  planningCommand: "/planning/command",
  vehicleCommand: "/vehicle/command",
  vehicle: "/vehicle/state",
  drive: "/drive/status",
  steering: "/steering/status",
  odometry: "/localization/odometry",
  vehicleOdometry: "/localization/kinematic_state",
  globalMarkers: "/debug/global_route_markers",
  sysidRoute: "/sysid/route_context",
  sysidTargetPath: "/sysid/target_path",
  sysidTargetSpeed: "/sysid/target_speed",
  sysidCommand: "/sysid/route_command",
  sysidOdometry: "/sysid/odometry",
  sysidGlobalMarkers: "/sysid/global_route_markers",
  trajectory: "/debug/decision_fox/trajectory",
  leftImage: "/perception/camera/left/wide/image_raw/compressed",
  rightImage: "/perception/camera/right/wide/image_raw/compressed",
  leftSourceImage: "/perception/camera/left/source/image_raw/compressed",
  rightSourceImage: "/perception/camera/right/source/image_raw/compressed",
} as const;

const LIVE_HISTORY_LIMIT = 600;
const OFFLINE_HISTORY_LIMIT = 6000;

export function initialModel(): MonitorModel {
  return {
    globalPath: [],
    waypoints: [],
    stopWaypoints: [],
    zonePaths: [],
    localPath: [],
    targetPath: [],
    trail: [],
    isReplay: false,
    didSeek: false,
    histories: {
      speedTarget: [],
      speedActual: [],
      steerTarget: [],
      steerActual: [],
    },
  };
}

export function asObject(value: unknown): RosObject | undefined {
  return typeof value === "object" && value != undefined && !Array.isArray(value)
    ? (value as RosObject)
    : undefined;
}

export function textField(value: unknown, key: string, fallback = ""): string {
  const field = asObject(value)?.[key];
  return typeof field === "string" ? field : fallback;
}

export function numberField(value: unknown, key: string): number | undefined {
  const field = asObject(value)?.[key];
  return typeof field === "number" && Number.isFinite(field) ? field : undefined;
}

export function boolField(value: unknown, key: string): boolean | undefined {
  const field = asObject(value)?.[key];
  return typeof field === "boolean" ? field : undefined;
}

export function arrayField(value: unknown, key: string): readonly unknown[] {
  const field = asObject(value)?.[key];
  return Array.isArray(field) ? field : [];
}

export function timeToSeconds(time: Time | undefined): number | undefined {
  if (time == undefined) {
    return undefined;
  }
  return time.sec + time.nsec * 1e-9;
}

function point(value: unknown): Point2 | undefined {
  const row = asObject(value);
  const x = row?.x;
  const y = row?.y;
  return typeof x === "number" && typeof y === "number" && Number.isFinite(x) && Number.isFinite(y)
    ? { x, y }
    : undefined;
}

function points(value: unknown): Point2[] {
  if (!Array.isArray(value)) {
    return [];
  }
  return value.map(point).filter((item): item is Point2 => item != undefined);
}

function nested(value: unknown, ...keys: string[]): unknown {
  let current = value;
  for (const key of keys) {
    current = asObject(current)?.[key];
  }
  return current;
}

function quaternionYaw(value: unknown): number {
  const q = asObject(value);
  const x = typeof q?.x === "number" ? q.x : 0;
  const y = typeof q?.y === "number" ? q.y : 0;
  const z = typeof q?.z === "number" ? q.z : 0;
  const w = typeof q?.w === "number" ? q.w : 1;
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

function odometryPose(message: unknown): EgoPose | undefined {
  const position = point(nested(message, "pose", "pose", "position"));
  if (position == undefined) {
    return undefined;
  }
  return {
    ...position,
    yaw: quaternionYaw(nested(message, "pose", "pose", "orientation")),
  };
}

function append(history: TimedValue[], item: TimedValue, limit = LIVE_HISTORY_LIMIT): void {
  const previous = history.length > 0 ? history[history.length - 1] : undefined;
  if (previous != undefined && item.time < previous.time - 1e-6) {
    history.length = 0;
  }
  history.push(item);
  if (history.length > limit) {
    history.splice(0, history.length - limit);
  }
}

function appendTrail(model: MonitorModel, ego: EgoPose): void {
  const previous = model.trail.length > 0 ? model.trail[model.trail.length - 1] : undefined;
  if (previous == undefined || Math.hypot(ego.x - previous.x, ego.y - previous.y) >= 0.1) {
    model.trail.push({ x: ego.x, y: ego.y });
    if (model.trail.length > 4000) {
      model.trail.splice(0, model.trail.length - 4000);
    }
  }
}

function byteArray(value: unknown): Uint8Array | undefined {
  if (value instanceof Uint8Array) {
    return new Uint8Array(value);
  }
  if (Array.isArray(value)) {
    return Uint8Array.from(value.filter((item): item is number => typeof item === "number"));
  }
  return undefined;
}

function cameraFrame(message: unknown): CameraFrame | undefined {
  const bytes = byteArray(asObject(message)?.data);
  if (bytes == undefined || bytes.length === 0) {
    return undefined;
  }
  return {
    bytes,
    format: textField(message, "format", "jpeg").toLowerCase(),
  };
}

function markerColor(message: unknown): string {
  const color = asObject(asObject(message)?.color);
  const channel = (key: string): number => {
    const value = color?.[key];
    return Math.round(255 * (typeof value === "number" ? Math.max(0, Math.min(1, value)) : 1));
  };
  return `rgba(${channel("r")}, ${channel("g")}, ${channel("b")}, 0.78)`;
}

function parseGlobalMarkers(message: unknown): {
  globalPath: Point2[];
  waypoints: Point2[];
  stopWaypoints: Point2[];
  zones: ZonePath[];
} {
  let globalPath: Point2[] = [];
  let waypoints: Point2[] = [];
  let stopWaypoints: Point2[] = [];
  const stoplineFallback: Point2[] = [];
  const zones: ZonePath[] = [];
  for (const rawMarker of arrayField(message, "markers")) {
    const namespace = textField(rawMarker, "ns");
    const markerPoints = points(asObject(rawMarker)?.points);
    if (namespace === "global_route_line" && markerPoints.length > 1) {
      globalPath = markerPoints;
    } else if (namespace === "global_route_waypoints") {
      waypoints = markerPoints;
    } else if (namespace === "global_route_brake_waypoints") {
      stopWaypoints = markerPoints;
    } else if (namespace === "global_route_stoplines") {
      for (let index = 0; index + 1 < markerPoints.length; index += 2) {
        const first = markerPoints[index];
        const second = markerPoints[index + 1];
        if (first != undefined && second != undefined) {
          stoplineFallback.push({
            x: 0.5 * (first.x + second.x),
            y: 0.5 * (first.y + second.y),
          });
        }
      }
    } else if (namespace.startsWith("route_zone_") && markerPoints.length > 1) {
      zones.push({
        name: namespace.slice("route_zone_".length).toUpperCase(),
        color: markerColor(rawMarker),
        points: markerPoints,
      });
    }
  }
  return {
    globalPath,
    waypoints,
    stopWaypoints: stopWaypoints.length > 0 ? stopWaypoints : stoplineFallback,
    zones,
  };
}

function parsePath(message: unknown): Point2[] {
  return arrayField(message, "poses")
    .map((pose) => point(nested(pose, "pose", "position")))
    .filter((item): item is Point2 => item != undefined);
}

export function handleMessage(model: MonitorModel, event: MessageEvent): void {
  const message = asObject(event.message);
  if (message == undefined) {
    return;
  }
  const time = timeToSeconds(event.receiveTime) ?? 0;
  switch (event.topic) {
    case TOPICS.behavior:
      model.behavior = message;
      break;
    case TOPICS.scenario:
      model.scenario = message;
      break;
    case TOPICS.route:
    case TOPICS.sysidRoute:
      model.route = message;
      model.localPath = points(message.local_path_points);
      break;
    case TOPICS.routeReady:
      model.routeReady = boolField(message, "data");
      break;
    case TOPICS.readiness:
      model.readiness = message;
      model.readinessTime = time;
      break;
    case TOPICS.scene:
      model.scene = message;
      break;
    case TOPICS.targetPath:
    case TOPICS.sysidTargetPath:
      model.targetPath = points(message.points);
      break;
    case TOPICS.targetSpeed:
    case TOPICS.sysidTargetSpeed:
      model.targetSpeed = message;
      break;
    case TOPICS.rawCommand:
      model.rawCommand = message;
      break;
    case TOPICS.planningCommand:
      model.planningCommand = message;
      break;
    case TOPICS.vehicleCommand:
      model.vehicleCommand = message;
      appendScalarHistory(model.histories, event.topic, message, time);
      break;
    case TOPICS.sysidCommand:
      model.planningCommand = message;
      appendScalarHistory(model.histories, event.topic, message, time);
      break;
    case TOPICS.vehicle:
      model.vehicle = message;
      appendScalarHistory(model.histories, event.topic, message, time);
      break;
    case TOPICS.drive:
      model.drive = message;
      appendScalarHistory(model.histories, event.topic, message, time);
      break;
    case TOPICS.steering:
      model.steering = message;
      appendScalarHistory(model.histories, event.topic, message, time);
      break;
    case TOPICS.odometry:
    case TOPICS.vehicleOdometry:
    case TOPICS.sysidOdometry: {
      const ego = odometryPose(message);
      if (ego != undefined) {
        model.ego = ego;
        appendTrail(model, ego);
      }
      break;
    }
    case TOPICS.globalMarkers:
    case TOPICS.sysidGlobalMarkers: {
      const parsed = parseGlobalMarkers(message);
      if (parsed.globalPath.length > 1) {
        model.globalPath = parsed.globalPath;
      }
      if (parsed.waypoints.length > 0) {
        model.waypoints = parsed.waypoints;
      }
      model.stopWaypoints = parsed.stopWaypoints;
      if (parsed.zones.length > 0) {
        model.zonePaths = parsed.zones;
      }
      break;
    }
    case TOPICS.trajectory: {
      const path = parsePath(message);
      if (path.length > 0) {
        model.trail = path;
      }
      break;
    }
    case TOPICS.leftImage:
      model.leftImage = cameraFrame(message);
      break;
    case TOPICS.rightImage:
      model.rightImage = cameraFrame(message);
      break;
    case TOPICS.leftSourceImage:
      model.leftSourceImage = cameraFrame(message);
      break;
    case TOPICS.rightSourceImage:
      model.rightSourceImage = cameraFrame(message);
      break;
  }
}

export function appendScalarHistory(
  histories: Histories,
  topic: string,
  message: RosObject,
  time: number,
  limit = LIVE_HISTORY_LIMIT,
): void {
  const add = (history: TimedValue[], value: number | undefined): void => {
    if (value != undefined) {
      append(history, { time, value }, limit);
    }
  };
  if (
    topic === TOPICS.vehicleCommand || topic === TOPICS.sysidCommand
  ) {
    add(histories.speedTarget, numberField(message, "speed_target_mps"));
    add(histories.steerTarget, numberField(message, "steering_target_rad"));
  } else if (topic === TOPICS.drive) {
    add(histories.speedActual, numberField(message, "speed_mps"));
  } else if (topic === TOPICS.vehicle) {
    if (histories.speedActual.length === 0) {
      add(histories.speedActual, numberField(message, "speed_mps"));
    }
    if (histories.steerActual.length === 0) {
      add(histories.steerActual, numberField(message, "steering_rad"));
    }
  } else if (topic === TOPICS.steering) {
    add(histories.steerActual, numberField(message, "actual_wheel_angle_rad"));
  }
}

export function loadOfflineSeries(topic: string, events: readonly MessageEvent[]): TimedValue[] {
  const values: TimedValue[] = [];
  for (const event of events) {
    const message = asObject(event.message);
    if (message == undefined) {
      continue;
    }
    const time = timeToSeconds(event.receiveTime);
    let value: number | undefined;
    if (topic === TOPICS.vehicleCommand) {
      value = numberField(message, "speed_target_mps");
    } else if (topic === `${TOPICS.vehicleCommand}:steer`) {
      value = numberField(message, "steering_target_rad");
    } else if (topic === TOPICS.drive) {
      value = numberField(message, "speed_mps");
    } else if (topic === TOPICS.steering) {
      value = numberField(message, "actual_wheel_angle_rad");
    }
    if (time != undefined && value != undefined) {
      append(values, { time, value }, OFFLINE_HISTORY_LIMIT);
    }
  }
  return values;
}

export function cloneForRender(model: MonitorModel): MonitorModel {
  return {
    ...model,
    globalPath: [...model.globalPath],
    waypoints: [...model.waypoints],
    stopWaypoints: [...model.stopWaypoints],
    zonePaths: model.zonePaths.map((zone) => ({ ...zone, points: [...zone.points] })),
    localPath: [...model.localPath],
    targetPath: [...model.targetPath],
    trail: [...model.trail],
    histories: {
      speedTarget: [...model.histories.speedTarget],
      speedActual: [...model.histories.speedActual],
      steerTarget: [...model.histories.steerTarget],
      steerActual: [...model.histories.steerActual],
    },
  };
}
