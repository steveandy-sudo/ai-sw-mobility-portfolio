import { ReactElement } from "react";

import { numberField, textField } from "./model";
import { MonitorModel, ObjectBox, Point2 } from "./types";

// Vehicle body around the rear axle (base_footprint): Decision vehicle_front_overhang_m and width.
const FRONT_M = 1.98;
const REAR_M = 0.62;
const HALF_WIDTH_M = 0.59;
// Decision StoppingModel (decision_pipeline.yaml brake_delay_sec, brake_decel_mps2).
const BRAKE_DELAY_S = 0.7;
const BRAKE_DECEL_MPS2 = 3.9;
const AHEAD_M = 34;
const BEHIND_M = 4;
const SIDE_M = 5.5;
// kaiev26_msgs/RoadSegment types.
const WHITE_LINES = new Set([1, 2]);
const YELLOW_LINES = new Set([3, 4]);
const STOP_LINE = 5;
// Perception gateway obstacle_max_size_m: larger boxes are not obstacles for the planner either.
const MAX_OBJECT_SIZE_M = 2.5;

type PathKind = { label: string; color: string };

// Decision target path sources (planning_core.make_sequence_bypass and friends).
export function pathKind(source: string): PathKind {
  if (source.startsWith("forced_sequential")) {
    return { label: "강제 S자 (여유 미확보)", color: "#ff6266" };
  }
  if (source.startsWith("committed_sequential")) {
    return { label: "확정 회피 경로 유지", color: "#b48cff" };
  }
  if (source.startsWith("relaxed_sequential")) {
    return { label: "회피 경로 (여유 완화)", color: "#f4b84a" };
  }
  if (source.startsWith("sequential_bypass")) {
    return { label: "회피 경로", color: "#43db91" };
  }
  if (source.includes("rejoin")) {
    return { label: "전역경로 복귀", color: "#9fb4ba" };
  }
  if (source.startsWith("global_route")) {
    return { label: "전역경로", color: "#20d7eb" };
  }
  if (source.startsWith("longitudinal_stop")) {
    return { label: "전역경로 (정지 목표)", color: "#20d7eb" };
  }
  if (source.startsWith("previous_safe")) {
    return { label: "직전 안전 경로 유지", color: "#f4b84a" };
  }
  return { label: source === "" ? "경로 대기" : source, color: "#dfe9eb" };
}

function stoppingDistance(speed: number): number {
  return speed * BRAKE_DELAY_S + (speed * speed) / (2 * BRAKE_DECEL_MPS2);
}

function signed(value: number, digits: number): string {
  return `${value >= 0 ? "+" : ""}${value.toFixed(digits)}`;
}

function boxCorners(box: ObjectBox): Point2[] {
  const c = Math.cos(box.yaw);
  const s = Math.sin(box.yaw);
  return [
    [box.length / 2, box.width / 2],
    [box.length / 2, -box.width / 2],
    [-box.length / 2, -box.width / 2],
    [-box.length / 2, box.width / 2],
  ].map(([dx = 0, dy = 0]) => ({ x: box.x + c * dx - s * dy, y: box.y + s * dx + c * dy }));
}

export function LocalView({ model }: { model: MonitorModel }): ReactElement {
  const width = 300;
  const height = 520;
  // base_footprint: x forward (screen up), y left (screen left).
  const sx = (y: number): number => width / 2 - (y / SIDE_M) * (width / 2);
  const sy = (x: number): number => height - ((x + BEHIND_M) / (AHEAD_M + BEHIND_M)) * height;
  const inView = (p: Point2): boolean => p.x > -BEHIND_M - 2 && p.x < AHEAD_M + 2 && Math.abs(p.y) < SIDE_M + 2;
  const line = (items: Point2[]): string =>
    items
      .filter(inView)
      .map((p, index) => `${index === 0 ? "M" : "L"}${sx(p.y).toFixed(1)},${sy(p.x).toFixed(1)}`)
      .join(" ");

  const behavior = textField(model.behavior, "active_behavior");
  const fsm = textField(model.behavior, "fsm_state");
  const obstacleMode = behavior === "OBSTACLE" || model.targetPathSource.includes("sequential");
  const kind = pathKind(model.targetPathSource);
  const cte = numberField(model.route, "cross_track_error");
  const speed = Math.abs(numberField(model.vehicle, "speed_mps") ?? numberField(model.drive, "speed_mps") ?? 0);
  const rearToLine = numberField(model.route, "distance_to_stopline");
  const lineAhead = rearToLine != undefined && rearToLine < 60 ? rearToLine : undefined;
  const frontGap = lineAhead == undefined ? undefined : lineAhead - FRONT_M;
  const stopped = speed < 0.1;
  const predicted = frontGap == undefined ? undefined : frontGap - (stopped ? 0 : stoppingDistance(speed));
  const objects = obstacleMode
    ? model.objects.filter((box) => box.x > -BEHIND_M && box.x < AHEAD_M && Math.abs(box.y) < SIDE_M
        && Math.max(box.length, box.width) <= MAX_OBJECT_SIZE_M)
    : [];
  const ticks = [0, 10, 20, 30];

  return (
    <section className="local-view">
      <div className="local-head">
        <strong>CTE: {cte == undefined ? "--" : `${signed(cte, 2)} m`}</strong>
        <span style={{ color: kind.color }}>{kind.label}</span>
      </div>
      {frontGap != undefined && (
        <div className="local-stop">
          정지선 앞범퍼 {frontGap.toFixed(1)} m ·{" "}
          {stopped ? "정지 위치" : "지금 제동 시 예상"}{" "}
          <b className={predicted != undefined && predicted < 0 ? "status-stop" : "status-good"}>
            {predicted == undefined ? "--" : predicted >= 0 ? `선 앞 ${predicted.toFixed(1)} m` : `선 넘음 ${(-predicted).toFixed(1)} m`}
          </b>
        </div>
      )}
      {obstacleMode && (
        <div className="local-stop">
          {behavior} · {fsm} · 드럼 박스 {objects.length}개
        </div>
      )}
      <svg className="local-svg" viewBox={`0 0 ${width} ${height}`} preserveAspectRatio="xMidYMid meet">
        <rect x="0" y="0" width={width} height={height} fill="#0b1012" />
        {ticks.map((tick) => (
          <g key={tick}>
            <line x1="0" x2={width} y1={sy(tick)} y2={sy(tick)} stroke="#1c2629" />
            <text className="plot-label" x="4" y={sy(tick) - 3}>{tick} m</text>
          </g>
        ))}
        {model.laneLines.map((lane, index) => {
          const color = lane.type === STOP_LINE ? "#ff6266" : YELLOW_LINES.has(lane.type) ? "#f4c542" : WHITE_LINES.has(lane.type) ? "#e8eef0" : "#7d8c91";
          const d = line(lane.points);
          return d === "" ? undefined : <path key={`lane-${index}`} d={d} fill="none" stroke={color} strokeWidth={lane.type === STOP_LINE ? 5 : 3} opacity="0.85" />;
        })}
        {lineAhead != undefined && lineAhead < AHEAD_M && (
          <g>
            <line x1={sx(2.5)} x2={sx(-2.5)} y1={sy(lineAhead)} y2={sy(lineAhead)} stroke="#ff454b" strokeWidth="6" />
            <text className="local-label" x={sx(-2.6) + 4} y={sy(lineAhead) + 4}>정지선</text>
          </g>
        )}
        {model.localPath.length > 1 && <path d={line(model.localPath)} fill="none" stroke="#20d7eb" strokeWidth="2" strokeDasharray="6 5" opacity="0.7" />}
        {model.targetPath.length > 1 && <path d={line(model.targetPath)} fill="none" stroke={kind.color} strokeWidth="4" />}
        {objects.map((box, index) => (
          <polygon
            key={`obj-${index}`}
            points={boxCorners(box).map((p) => `${sx(p.y).toFixed(1)},${sy(p.x).toFixed(1)}`).join(" ")}
            fill="rgba(255, 140, 60, 0.55)"
            stroke="#ff9d4a"
            strokeWidth="2"
          />
        ))}
        <rect
          x={sx(HALF_WIDTH_M)}
          y={sy(FRONT_M)}
          width={sx(-HALF_WIDTH_M) - sx(HALF_WIDTH_M)}
          height={sy(-REAR_M) - sy(FRONT_M)}
          fill="#20d7eb"
          stroke="#e9fdff"
          strokeWidth="2"
          rx="3"
        />
      </svg>
      <div className="local-legend">
        <span style={{ color: "#e8eef0" }}>흰 차선</span>
        <span style={{ color: "#f4c542" }}>중앙선</span>
        <span style={{ color: "#20d7eb" }}>전역경로</span>
        <span style={{ color: kind.color }}>판단 경로</span>
        {obstacleMode && <span style={{ color: "#ff9d4a" }}>드럼(LiDAR)</span>}
        <span>가로 2배 확대</span>
      </div>
    </section>
  );
}
