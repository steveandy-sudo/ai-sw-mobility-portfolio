import { PanelExtensionContext, Time } from "@foxglove/extension";
import { PointerEvent, ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

import { FsmCard, breakable } from "./FsmCard";
import { LocalView } from "./LocalView";
import { SignalClock } from "./SignalClock";
import {
  TOPICS,
  arrayField,
  asObject,
  boolField,
  cloneForRender,
  handleMessage,
  initialModel,
  loadOfflineSeries,
  numberField,
  textField,
  timeToSeconds,
} from "./model";
import { styles } from "./styles";
import {
  CameraFrame,
  EgoPose,
  MonitorModel,
  Point2,
  RosObject,
  TimedValue,
} from "./types";

// Panel redraw rate; Foxglove delivers frames faster (up to 60 Hz) during playback.
const RENDER_HZ = 20;

// Decision modules (D18): route follow, the stop-point machine per event kind, the obstacle
// behavior and the input-loss rule. Behavior names are BehaviorDecision.active_behavior.
const FSM = [
  { key: "lane", name: "RouteFollow" },
  { key: "traffic", name: "SignalStopPoint" },
  { key: "obstacle", name: "ObstacleBehavior" },
  { key: "recovery", name: "InputLoss" },
  { key: "static", name: "TimedStopPoint" },
  { key: "finish", name: "FinishStopPoint" },
] as const;

const BEHAVIOR_FSM: Record<string, string> = {
  GLOBAL_ROUTE_FOLLOW: "lane",
  TRAFFIC_LIGHT: "traffic",
  OBSTACLE: "obstacle",
  RECOVERY: "recovery",
  STATIC_STOP: "static",
  FINISH: "finish",
};

const STATE_LABELS: Record<string, string> = {
  ROUTE_FOLLOW: "전역 경로의 지역 구간을 추종 중",
  SIGNAL_APPROACH_GO: "신호 정지선 접근, 지금은 통과 허용",
  SIGNAL_APPROACH_STOP: "신호 정지선 접근, 지금은 통과 불허",
  SIGNAL_PRE_BRAKE: "제동 지점 앞에서 미리 감속 중",
  SIGNAL_BRAKE: "신호 정지선에서 제동 중",
  SIGNAL_WAIT: "신호 정지선에 서서 허용 대기",
  SIGNAL_PASS: "통과 확정, 같은 신호로 다시 제동하지 않음",
  TIMED_STOP_APPROACH: "정지 미션 정지선 접근",
  TIMED_STOP_PRE_BRAKE: "제동 지점 앞에서 미리 감속 중",
  TIMED_STOP_BRAKE: "정지 미션 정지선에서 제동 중",
  TIMED_STOP_WAIT: "정지 시간 채우는 중",
  TIMED_STOP_PASS: "정지 시간을 채우고 출발",
  FINAL_SPRINT: "신호 없는 마지막 구간(F6) 고정 속도 주행",
  FINISH_APPROACH: "결승선 앞 감속 중",
  FINISH_BRAKE: "결승선에서 제동 중",
  FINISH_STOPPED: "결승 정지 완료",
  OBSTACLE_APPROACH: "장애물 구간 앞에서 속도를 낮추는 중",
  OBSTACLE_LANE_KEEP: "장애물 구간 상한 속도로 차선 주행 중",
  OBSTACLE_BYPASS: "확정한 회피 경로 추종 중",
  OBSTACLE_BYPASS_FORCED: "여유가 모자란 강제 회피 경로 추종 중",
  OBSTACLE_RETURN: "마지막 줄을 지나 경로로 복귀 중",
  OBSTACLE_NO_PATH: "회피 경로가 없어 저속으로 이어 가는 중",
  INPUT_LOSS_COAST: "입력 끊김, 브레이크 없이 속도 0",
  INPUT_LOSS_HOLD_BRAKE: "입력 끊김, 걸려 있던 브레이크 유지",
  INPUT_LOSS_BRAKE: "입력 끊김 1초 경과, 정지까지 제동",
  ESTOP: "E-Stop 정지 요청",
  MANUAL: "수동주행 모드로 자율 명령 정지",
};

const HEALTH_LABELS: Record<string, string> = {
  "PRE-FLIGHT": "전체 출발 조건",
  "/gnss/fix": "GNSS RTK FIX",
  "/gnss/fix_velocity": "GNSS 속도·헤딩",
  "/localization/odometry": "Localization",
  "/vehicle/state": "차량 상태·모드",
  "/drive/status": "구동 액추에이터",
  "/steering/status": "조향 액추에이터",
  "/brake/status": "브레이크 상태",
  "/planning/route_context": "경로 정합",
  "/planning/target_path": "목표 경로 발행",
  "/planning/target_speed": "목표 속도 발행",
  "/planning/command": "TUI 최종 명령",
  "/vehicle/command": "차량 명령 중계",
};

function fmt(value: number | undefined, digits: number, unit: string): string {
  return value == undefined ? "--" : `${value.toFixed(digits)} ${unit}`;
}

function commandLabel(command: RosObject | undefined): string {
  const speed = numberField(command, "speed_target_mps");
  const steering = degrees(numberField(command, "steering_target_rad"));
  const brake = boolField(command, "brake_engage");
  if (speed == undefined && steering == undefined && brake == undefined) {
    return "--";
  }
  return `${fmt(speed, 1, "m/s")} · ${fmt(steering, 0, "deg")} · ${brake === true ? "BRAKE" : "FREE"}`;
}

function trafficStateLabel(state: number | undefined): string {
  return state == undefined
    ? "--"
    : (["UNKNOWN", "RED", "YELLOW", "GREEN", "ARROW"][state] ?? `STATE ${state}`);
}

function holdTiming(behavior: RosObject | undefined): { elapsed: number; total: number } | undefined {
  if (textField(behavior, "active_behavior") !== "STATIC_STOP" || textField(behavior, "fsm_state") !== "TIMED_STOP_WAIT") {
    return undefined;
  }
  const match = /([0-9.]+)\/([0-9.]+)s/.exec(textField(behavior, "selected_reason"));
  if (match == undefined) {
    return undefined;
  }
  const elapsed = Number(match[1]);
  const total = Number(match[2]);
  return Number.isFinite(elapsed) && Number.isFinite(total) && total > 0
    ? { elapsed, total }
    : undefined;
}

// Decision publishes 1e6 m (INF_DISTANCE) when there is no stop line ahead.
function finiteDistance(value: number | undefined): number | undefined {
  return value == undefined || value >= 1e5 ? undefined : value;
}

function degrees(value: number | undefined): number | undefined {
  return value == undefined ? undefined : (value * 180) / Math.PI;
}

function clockLabel(seconds: number | undefined): string {
  if (seconds == undefined) {
    return "--:--:--.---";
  }
  if (seconds < 86_400 * 2) {
    const hour = Math.floor(seconds / 3600);
    const minute = Math.floor((seconds % 3600) / 60);
    const second = seconds % 60;
    return `${String(hour).padStart(2, "0")}:${String(minute).padStart(2, "0")}:${second.toFixed(3).padStart(6, "0")}`;
  }
  const date = new Date(seconds * 1000);
  const milliseconds = String(date.getMilliseconds()).padStart(3, "0");
  return `${date.toLocaleTimeString("ko-KR", { hour12: false })}.${milliseconds}`;
}

function baseToMap(points: Point2[], ego: EgoPose | undefined): Point2[] {
  if (ego == undefined) {
    return [];
  }
  const cosine = Math.cos(ego.yaw);
  const sine = Math.sin(ego.yaw);
  return points.map((point) => ({
    x: ego.x + cosine * point.x - sine * point.y,
    y: ego.y + sine * point.x + cosine * point.y,
  }));
}

type Bounds = { minX: number; maxX: number; minY: number; maxY: number };

function fitBounds(points: Point2[], margin = 5): Bounds {
  if (points.length === 0) {
    return { minX: -10, maxX: 10, minY: -10, maxY: 10 };
  }
  const xs = points.map((point) => point.x);
  const ys = points.map((point) => point.y);
  return {
    minX: Math.min(...xs) - margin,
    maxX: Math.max(...xs) + margin,
    minY: Math.min(...ys) - margin,
    maxY: Math.max(...ys) + margin,
  };
}

function MapPanel({ model, overview }: { model: MonitorModel; overview: boolean }): ReactElement {
  const width = 720;
  const height = 390;
  const localPath = baseToMap(model.localPath, model.ego);
  const targetPath = baseToMap(model.targetPath, model.ego);
  const all = [...model.globalPath, ...model.trail, ...localPath, ...targetPath];
  const bounds =
    overview || model.ego == undefined
      ? fitBounds(all, 8)
      : {
          minX: model.ego.x - 20,
          maxX: model.ego.x + 20,
          minY: model.ego.y - 17,
          maxY: model.ego.y + 17,
        };
  const spanX = Math.max(1, bounds.maxX - bounds.minX);
  const spanY = Math.max(1, bounds.maxY - bounds.minY);
  const project = (point: Point2): Point2 => ({
    x: ((point.x - bounds.minX) / spanX) * width,
    y: height - ((point.y - bounds.minY) / spanY) * height,
  });
  const pathData = (items: Point2[]): string =>
    items
      .map((item, index) => {
        const projected = project(item);
        return `${index === 0 ? "M" : "L"}${projected.x.toFixed(1)},${projected.y.toFixed(1)}`;
      })
      .join(" ");
  const ego = model.ego == undefined ? undefined : project(model.ego);
  const currentZone = textField(model.route, "current_zone", "UNKNOWN_ZONE");
  const nextZone = textField(model.route, "next_zone", "--");
  const grid = Array.from({ length: 9 }, (_, index) => index);

  return (
    <section className="map-panel">
      <div className="panel-title">{overview ? "Route overview / zone context" : "Local route tracking"}</div>
      <div className="panel-subtitle">{currentZone} · NEXT {nextZone}</div>
      <svg className="map-svg" viewBox={`0 0 ${width} ${height}`} preserveAspectRatio="none">
        {grid.map((index) => (
          <g key={index} stroke="#1e292d" strokeWidth="1">
            <line x1={(index * width) / 8} y1="0" x2={(index * width) / 8} y2={height} />
            <line x1="0" y1={(index * height) / 8} x2={width} y2={(index * height) / 8} />
          </g>
        ))}
        {model.zonePaths.map((zone) => (
          <path key={`${zone.name}-${zone.points.length}`} d={pathData(zone.points)} fill="none" stroke={zone.color} strokeWidth="18" opacity="0.33" />
        ))}
        {model.globalPath.length > 1 && <path d={pathData(model.globalPath)} fill="none" stroke="#16d6ec" strokeWidth="4" />}
        {model.waypoints.map((waypoint, index) => {
          const projected = project(waypoint);
          return <circle key={`waypoint-${index}`} cx={projected.x} cy={projected.y} r="1.7" fill="#f4f7f7" opacity="0.92" />;
        })}
        {model.stopWaypoints.map((waypoint, index) => {
          const projected = project(waypoint);
          return <circle key={`stop-waypoint-${index}`} cx={projected.x} cy={projected.y} r="5.2" fill="#ff454b" stroke="#fff4f4" strokeWidth="1.6" />;
        })}
        {targetPath.length > 1 && <path d={pathData(targetPath)} fill="none" stroke="#ff5d62" strokeWidth="5" />}
        {localPath.length > 1 && <path d={pathData(localPath)} fill="none" stroke="#f5ad42" strokeWidth="3" strokeDasharray="8 5" />}
        {model.trail.length > 1 && <path d={pathData(model.trail)} fill="none" stroke="#f4f7f7" strokeWidth="3" opacity="0.92" />}
        {ego != undefined && (
          <g transform={`rotate(${(-model.ego!.yaw * 180) / Math.PI} ${ego.x} ${ego.y})`}>
            <rect x={ego.x - 14} y={ego.y - 8} width="28" height="16" rx="2" fill="#20d7eb" stroke="#e9fdff" strokeWidth="2" />
            <path d={`M${ego.x + 14},${ego.y} L${ego.x + 23},${ego.y}`} stroke="#20d7eb" strokeWidth="3" />
          </g>
        )}
      </svg>
      <div className="map-legend">
        <span className="legend-line" style={{ color: "#16d6ec" }}><i />전역 경로</span>
        <span className="legend-point"><i className="legend-dot waypoint-dot" />WP</span>
        <span className="legend-point"><i className="legend-dot stop-dot" />제동 WP</span>
        <span className="legend-line" style={{ color: "#ff5d62" }}><i />추종 경로</span>
        <span className="legend-line" style={{ color: "#f4f7f7" }}><i />실제 궤적</span>
      </div>
    </section>
  );
}

function diagnosticDetail(status: RosObject): string {
  const detail = arrayField(status, "values")
    .map(asObject)
    .find((item) => textField(item, "key") === "detail");
  return textField(detail, "value", textField(status, "message", "상태 대기"));
}

function HealthPanel({ model }: { model: MonitorModel }): ReactElement {
  const isReplay = model.isReplay;
  const statuses = arrayField(model.readiness, "status")
    .map(asObject)
    .filter((item): item is RosObject => item != undefined);
  const byName = new Map(statuses.map((status) => [textField(status, "name"), status]));
  const unavailable = statuses.length === 0;
  const stale = model.readinessTime == undefined || model.currentTime == undefined
    ? false
    : model.currentTime - model.readinessTime > 1.5;
  const aggregate = byName.get("PRE-FLIGHT");
  const aggregateLevel = unavailable ? 1 : stale ? 3 : numberField(aggregate, "level");
  const aggregateLabel = unavailable
    ? "WAITING"
    : aggregateLevel === 0
    ? "ALL GOOD"
    : aggregateLevel === 2 || aggregateLevel === 3
    ? "FAULT"
    : "CHECKING";
  const scenario = textField(model.scenario, "data", "Decision Test 대기");
  const orderedNames = Object.keys(HEALTH_LABELS).filter((name) => name !== "PRE-FLIGHT");

  return (
    <section className="health-panel">
      <div className="health-heading">
        <div>
          <span className="eyebrow">Decision test readiness / runtime health</span>
          <strong>{scenario}</strong>
        </div>
        <div className={`health-overall health-level-${aggregateLevel ?? 1}`}>
          <i />{aggregateLabel}
        </div>
      </div>
      <div className="health-grid">
        {orderedNames.map((name) => {
          const status = byName.get(name);
          const level = stale ? 3 : numberField(status, "level");
          const state = level === 0 ? "GOOD" : level === 2 || level === 3 ? "FAULT" : "WAIT";
          return (
            <div className={`health-row health-level-${level ?? 1}`} key={name} title={status == undefined ? "상태 메시지 대기" : diagnosticDetail(status)}>
              <i />
              <span>{HEALTH_LABELS[name]}</span>
              <b>{state}</b>
            </div>
          );
        })}
      </div>
      <div className="health-foot">
        <span>상태 원문은 각 항목에 마우스를 올려 확인</span>
        <div className="health-stat"><span>ROS TIME</span><strong>{clockLabel(model.currentTime)}</strong></div>
        <div className="health-stat"><span>DATA SOURCE</span><strong className={isReplay ? "replay-pill" : "live-pill"}>{isReplay ? "MCAP REPLAY" : "LIVE"}</strong></div>
      </div>
    </section>
  );
}

function imageMime(format: string): string {
  if (format.includes("png")) {
    return "image/png";
  }
  if (format.includes("webp")) {
    return "image/webp";
  }
  return "image/jpeg";
}

function CameraPanel({
  side,
  frame,
}: {
  side: "LEFT" | "RIGHT";
  frame: CameraFrame | undefined;
}): ReactElement {
  const [url, setUrl] = useState<string>();
  useEffect(() => {
    if (frame == undefined) {
      setUrl(undefined);
      return;
    }
    const imageBuffer = new ArrayBuffer(frame.bytes.byteLength);
    new Uint8Array(imageBuffer).set(frame.bytes);
    const next = URL.createObjectURL(new Blob([imageBuffer], { type: imageMime(frame.format) }));
    setUrl(next);
    return () => {
      URL.revokeObjectURL(next);
    };
  }, [frame]);
  return (
    <section className="camera-panel">
      <div className="panel-title">{side} CAMERA</div>
      <div className="camera-media">
        {url == undefined ? (
          <div className="camera-empty">CAMERA WAITING<br />/perception/camera/{side.toLowerCase()}/wide or source</div>
        ) : (
          <img src={url} />
        )}
      </div>
    </section>
  );
}

function nearestValue(series: TimedValue[], time: number | undefined): number | undefined {
  if (series.length === 0 || time == undefined) {
    return series.length > 0 ? series[series.length - 1]?.value : undefined;
  }
  let best = series[0];
  for (const item of series) {
    if (best == undefined || Math.abs(item.time - time) < Math.abs(best.time - time)) {
      best = item;
    }
  }
  return best?.value;
}

function secondsToTime(seconds: number): Time {
  const sec = Math.floor(seconds);
  return { sec, nsec: Math.max(0, Math.round((seconds - sec) * 1e9)) };
}

function TimePlot({
  title,
  unit,
  target,
  actual,
  currentTime,
  seek,
  transform = (value) => value,
}: {
  title: string;
  unit: string;
  target: TimedValue[];
  actual: TimedValue[];
  currentTime: number | undefined;
  seek: ((time: Time) => void) | undefined;
  transform?: (value: number) => number;
}): ReactElement {
  const width = 1000;
  const height = 160;
  const padding = { left: 42, right: 12, top: 10, bottom: 22 };
  const transformedTarget = target.map((item) => ({ ...item, value: transform(item.value) }));
  const transformedActual = actual.map((item) => ({ ...item, value: transform(item.value) }));
  const all = [...transformedTarget, ...transformedActual];
  const minTime = all.length > 0 ? Math.min(...all.map((item) => item.time)) : (currentTime ?? 0) - 10;
  const maxTime = all.length > 1 ? Math.max(...all.map((item) => item.time)) : currentTime ?? minTime + 10;
  const spanTime = Math.max(0.1, maxTime - minTime);
  const values = all.map((item) => item.value);
  let minValue = values.length > 0 ? Math.min(...values) : -1;
  let maxValue = values.length > 0 ? Math.max(...values) : 1;
  const margin = Math.max(0.2, (maxValue - minValue) * 0.16);
  minValue -= margin;
  maxValue += margin;
  const spanValue = Math.max(0.1, maxValue - minValue);
  const x = (time: number): number => padding.left + ((time - minTime) / spanTime) * (width - padding.left - padding.right);
  const y = (value: number): number => padding.top + (1 - (value - minValue) / spanValue) * (height - padding.top - padding.bottom);
  const line = (series: TimedValue[]): string => series.map((item, index) => `${index === 0 ? "M" : "L"}${x(item.time).toFixed(1)},${y(item.value).toFixed(1)}`).join(" ");
  const playheadX = currentTime == undefined ? undefined : x(Math.max(minTime, Math.min(maxTime, currentTime)));
  const seekAt = (event: PointerEvent<SVGSVGElement>): void => {
    if (seek == undefined) {
      return;
    }
    const bounds = event.currentTarget.getBoundingClientRect();
    const ratio = Math.max(0, Math.min(1, (event.clientX - bounds.left) / bounds.width));
    seek(secondsToTime(minTime + ratio * spanTime));
  };
  const targetNow = nearestValue(transformedTarget, currentTime);
  const actualNow = nearestValue(transformedActual, currentTime);
  return (
    <section className="plot-panel">
      <div className="plot-header">
        <h3>{title}</h3>
        <span><b style={{ color: "#20d7eb" }}>목표 {fmt(targetNow, 2, unit)}</b> · <b style={{ color: "#f2f5f5" }}>실제 {fmt(actualNow, 2, unit)}</b></span>
      </div>
      <svg
        className="plot-svg"
        viewBox={`0 0 ${width} ${height}`}
        preserveAspectRatio="none"
        onPointerDown={seekAt}
        onPointerMove={(event) => {
          if (event.buttons === 1) {
            seekAt(event);
          }
        }}
      >
        {[0, 1, 2, 3, 4].map((index) => {
          const rowY = padding.top + (index * (height - padding.top - padding.bottom)) / 4;
          return <line key={index} x1={padding.left} y1={rowY} x2={width - padding.right} y2={rowY} stroke="#2a3438" />;
        })}
        <text className="plot-label" x="4" y={padding.top + 6}>{maxValue.toFixed(1)}</text>
        <text className="plot-label" x="4" y={height - padding.bottom}>{minValue.toFixed(1)}</text>
        <path d={line(transformedTarget)} fill="none" stroke="#20d7eb" strokeWidth="2.5" />
        <path d={line(transformedActual)} fill="none" stroke="#f2f5f5" strokeWidth="2.3" />
        {playheadX != undefined && <line x1={playheadX} y1={padding.top} x2={playheadX} y2={height - padding.bottom} stroke="#ff6266" strokeWidth="2" />}
        <text className="plot-label" x={padding.left} y={height - 5}>{clockLabel(minTime)}</text>
        <text className="plot-label" textAnchor="end" x={width - padding.right} y={height - 5}>{clockLabel(maxTime)}</text>
      </svg>
    </section>
  );
}

function MonitorPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const modelRef = useRef<MonitorModel>(initialModel());
  const [model, setModel] = useState<MonitorModel>(() => cloneForRender(modelRef.current));
  // Signal cycle offset from the UTC 100 s grid (panel setting; 10/17 K-City measured 0 s).
  const [utcOffsetSec, setUtcOffsetSec] = useState<number>(() => {
    const saved = (context.initialState as { utcOffsetSec?: unknown } | undefined)?.utcOffsetSec;
    return typeof saved === "number" && Number.isFinite(saved) ? saved : 0;
  });

  useEffect(() => {
    context.updatePanelSettingsEditor({
      actionHandler: (action) => {
        if (action.action !== "update" || action.payload.path[1] !== "utcOffsetSec") {
          return;
        }
        const value = Number(action.payload.value);
        if (Number.isFinite(value)) {
          setUtcOffsetSec(value);
          context.saveState({ utcOffsetSec: value });
        }
      },
      nodes: {
        signal: {
          label: "신호 주기",
          fields: {
            utcOffsetSec: { label: "UTC 오프셋 (s)", input: "number", value: utcOffsetSec, step: 1 },
          },
        },
      },
    });
  }, [context, utcOffsetSec]);

  useLayoutEffect(() => {
    const subscriptions = Object.values(TOPICS).map((topic) => ({ topic }));
    context.subscribe(subscriptions);
    context.watch("currentFrame");
    context.watch("currentTime");
    context.watch("startTime");
    context.watch("endTime");
    context.watch("didSeek");
    context.setDefaultPanelTitle("KAIEV26 Decision Monitor");
    let lastRenderMs = 0;
    let renderTimer: ReturnType<typeof setTimeout> | undefined;
    context.onRender = (renderState, done) => {
      const current = modelRef.current;
      current.didSeek = renderState.didSeek === true;
      if (renderState.didSeek === true) {
        current.trail = [];
        current.leftImage = undefined;
        current.rightImage = undefined;
        current.leftSourceImage = undefined;
        current.rightSourceImage = undefined;
      }
      current.currentTime = timeToSeconds(renderState.currentTime) ?? current.currentTime;
      current.startTime = timeToSeconds(renderState.startTime) ?? current.startTime;
      current.endTime = timeToSeconds(renderState.endTime) ?? current.endTime;
      for (const event of renderState.currentFrame ?? []) {
        handleMessage(current, event);
      }
      // Messages are applied every frame; the React tree redraws at most RENDER_HZ, with a
      // trailing redraw so a paused or seeked view still shows the last frame.
      const now = performance.now();
      const wait = 1000 / RENDER_HZ - (now - lastRenderMs);
      if (renderTimer != undefined) {
        clearTimeout(renderTimer);
        renderTimer = undefined;
      }
      if (wait <= 0 || renderState.didSeek === true) {
        lastRenderMs = now;
        setModel(cloneForRender(current));
      } else {
        renderTimer = setTimeout(() => {
          renderTimer = undefined;
          lastRenderMs = performance.now();
          setModel(cloneForRender(modelRef.current));
        }, wait);
      }
      done();
    };

    const stops: Array<() => void> = [];
    const subscribeRange = (
      sourceTopic: string,
      historyKey: keyof MonitorModel["histories"],
      valueSelector: string,
    ): void => {
      const stop = context.subscribeMessageRange?.({
        topic: sourceTopic,
        onNewRangeIterator: async (iterator) => {
          let full: TimedValue[] = [];
          for await (const batch of iterator) {
            full = [...full, ...loadOfflineSeries(valueSelector, batch)];
            if (full.length > 6000) {
              full = full.slice(full.length - 6000);
            }
          }
          modelRef.current.histories[historyKey] = full;
          setModel(cloneForRender(modelRef.current));
        },
      });
      if (stop != undefined) {
        stops.push(stop);
      }
    };
    subscribeRange(TOPICS.vehicleCommand, "speedTarget", TOPICS.vehicleCommand);
    subscribeRange(TOPICS.vehicleCommand, "steerTarget", `${TOPICS.vehicleCommand}:steer`);
    subscribeRange(TOPICS.drive, "speedActual", TOPICS.drive);
    subscribeRange(TOPICS.steering, "steerActual", TOPICS.steering);
    modelRef.current.isReplay = stops.length > 0;
    return () => {
      for (const stop of stops) {
        stop();
      }
      context.onRender = undefined;
      if (renderTimer != undefined) {
        clearTimeout(renderTimer);
      }
    };
  }, [context]);

  const behavior = model.behavior;
  const route = model.route;
  const activeBehavior = textField(behavior, "active_behavior", "WAITING");
  const activeFsm = BEHAVIOR_FSM[activeBehavior];
  const fsmState = textField(behavior, "fsm_state", "WAITING");
  const selectedReason = textField(behavior, "selected_reason", "판단 데이터 대기");
  const hold = holdTiming(behavior);
  const stateDescription = hold != undefined ? selectedReason : STATE_LABELS[fsmState] ?? selectedReason;
  const finalCommand = model.vehicleCommand ?? model.planningCommand ?? model.rawCommand;
  const targetSpeed = numberField(finalCommand, "speed_target_mps") ?? numberField(model.targetSpeed, "target_speed_mps");
  const actualSpeed = numberField(model.drive, "speed_mps") ?? numberField(model.vehicle, "speed_mps");
  const targetSteer = degrees(numberField(finalCommand, "steering_target_rad"));
  const actualSteer = degrees(numberField(model.steering, "actual_wheel_angle_rad") ?? numberField(model.vehicle, "steering_rad"));
  const currentZone = textField(route, "current_zone", "UNKNOWN_ZONE");
  const nextZone = textField(route, "next_zone", "--");
  const isReplay = model.isReplay;
  const projectionValid = boolField(route, "route_projection_valid");
  const trafficState = numberField(model.scene, "traffic_light_state");
  const obstacleOnPath = boolField(model.scene, "obstacle_on_path");
  const obstacleDistance = numberField(model.scene, "front_obstacle_distance");
  const leftFrame = model.leftImage ?? model.leftSourceImage;
  const rightFrame = model.rightImage ?? model.rightSourceImage;
  const seekPlayback = !isReplay || context.seekPlayback == undefined
    ? undefined
    : (time: Time): void => {
        context.seekPlayback?.(time);
      };

  return (
    <div className="kai-fox">
      <style>{styles}</style>
      <main className="monitor-shell">
        <div className="monitor-grid">
          <div className="left-stack">
            <div className="map-grid">
              <HealthPanel model={model} />
              <MapPanel model={model} overview />
            </div>
            <section className="fsm-section">
              <div className="section-heading"><h2>행동 선택 · FSM</h2><span>활성 FSM은 현재 선택된 행동의 제어 상태를 표시</span></div>
              <div className="fsm-body">
                <div className="fsm-grid">
                  {FSM.map((fsm, index) => (
                    <FsmCard key={fsm.key} index={index + 1} name={fsm.name} active={activeFsm === fsm.key} state={fsmState} description={stateDescription} />
                  ))}
                </div>
                <LocalView model={model} />
              </div>
            </section>
          </div>
          <div className="right-stack">
            <div className="camera-grid">
              <CameraPanel side="LEFT" frame={leftFrame} />
              <CameraPanel side="RIGHT" frame={rightFrame} />
            </div>
            <section className="summary-panel">
              <div className="selected-state">
                <span className="eyebrow">Selected behavior / FSM state</span>
                <strong>{breakable(activeBehavior)} · {breakable(fsmState)}</strong>
                <p>{stateDescription}</p>
                {hold != undefined && (
                  <div className="hold-timer">
                    <div><span>정지 유지</span><strong>{hold.elapsed.toFixed(1)} / {hold.total.toFixed(1)} s</strong></div>
                    <i><b style={{ width: `${Math.min(100, 100 * hold.elapsed / hold.total)}%` }} /></i>
                  </div>
                )}
                <div className="command-chain">
                  <div><span>DECISION RAW</span><b title={commandLabel(model.rawCommand)}>{commandLabel(model.rawCommand)}</b></div>
                  <div><span>TUI / PLANNING</span><b title={commandLabel(model.planningCommand)}>{commandLabel(model.planningCommand)}</b></div>
                  <div><span>VEHICLE</span><b title={commandLabel(model.vehicleCommand)}>{commandLabel(model.vehicleCommand)}</b></div>
                </div>
                <SignalClock model={model} offsetSec={utcOffsetSec} />
              </div>
              <div className="motion-values">
                <div className="motion-value target"><span>목표 속도</span><strong>{fmt(targetSpeed, 2, "m/s")}</strong></div>
                <div className="motion-value"><span>실제 속도</span><strong>{fmt(actualSpeed, 2, "m/s")}</strong></div>
                <div className="motion-value steer target"><span>목표 조향각</span><strong>{fmt(targetSteer, 1, "deg")}</strong></div>
                <div className="motion-value steer"><span>실제 조향각</span><strong>{fmt(actualSteer, 1, "deg")}</strong></div>
                <div className="route-strip">
                  <div><span>CURRENT ZONE</span><strong>{currentZone}</strong></div>
                  <div><span>NEXT ZONE</span><strong>{nextZone}</strong></div>
                  <div><span>ROUTE READY</span><strong className={model.routeReady === true ? "status-good" : model.routeReady === false ? "status-stop" : ""}>{model.routeReady == undefined ? "--" : model.routeReady ? "GOOD" : "WAIT"}</strong></div>
                  <div><span>PROJECTION</span><strong className={projectionValid === true ? "status-good" : projectionValid === false ? "status-stop" : ""}>{projectionValid == undefined ? "--" : projectionValid ? "VALID" : "INVALID"}</strong></div>
                  <div><span>CTE</span><strong>{fmt(numberField(route, "cross_track_error"), 2, "m")}</strong></div>
                  <div><span>HEADING ERROR</span><strong>{fmt(degrees(numberField(route, "heading_error")), 1, "deg")}</strong></div>
                  <div><span>TRAFFIC</span><strong>{trafficStateLabel(trafficState)}</strong></div>
                  <div><span>STOP LINE</span><strong>{fmt(finiteDistance(numberField(model.scene, "stopline_distance")), 1, "m")}</strong></div>
                  <div><span>OBSTACLE</span><strong className={obstacleOnPath === true ? "status-stop" : obstacleOnPath === false ? "status-good" : ""}>{obstacleOnPath == undefined ? "--" : obstacleOnPath ? fmt(obstacleDistance, 1, "m") : "CLEAR"}</strong></div>
                </div>
              </div>
            </section>
            <TimePlot title="조향각 · TARGET / ACTUAL" unit="deg" target={model.histories.steerTarget} actual={model.histories.steerActual} currentTime={model.currentTime} seek={seekPlayback} transform={(value) => (value * 180) / Math.PI} />
            <TimePlot title="속도 · TARGET / ACTUAL" unit="m/s" target={model.histories.speedTarget} actual={model.histories.speedActual} currentTime={model.currentTime} seek={seekPlayback} />
          </div>
        </div>
      </main>
    </div>
  );
}

export function initMonitorPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<MonitorPanel context={context} />);
  return () => {
    root.unmount();
  };
}
