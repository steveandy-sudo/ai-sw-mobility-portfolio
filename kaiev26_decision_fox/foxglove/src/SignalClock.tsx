import { ReactElement } from "react";

import { arrayField, asObject, boolField, numberField, textField } from "./model";
import { MonitorModel, RosObject } from "./types";

// K-City signal programs (kaiev26_maps kcity_course_semantics.yaml traffic_signal_programs): every
// cycle starts at a multiple of 100 s of UTC (10/17 measurement, offset 0 s). Lamps per window.
const PERIOD_S = 100;
type Window = { from: number; to: number; lamps: string };
const PROGRAMS: Record<string, Window[]> = {
  "142": [{ from: 0, to: 32, lamps: "GA" }, { from: 32, to: 35, lamps: "Y" }, { from: 35, to: 100, lamps: "R" }],
  "143": [{ from: 0, to: 32, lamps: "GA" }, { from: 32, to: 35, lamps: "Y" }, { from: 35, to: 100, lamps: "R" }],
  "144": [{ from: 0, to: 37, lamps: "G" }, { from: 37, to: 40, lamps: "Y" }, { from: 40, to: 100, lamps: "R" }],
  "305": [{ from: 0, to: 40, lamps: "G" }, { from: 40, to: 47, lamps: "GA" }, { from: 47, to: 50, lamps: "Y" }, { from: 50, to: 100, lamps: "R" }],
  "318": [{ from: 0, to: 85, lamps: "R" }, { from: 85, to: 97, lamps: "GA" }, { from: 97, to: 100, lamps: "Y" }],
  "322": [{ from: 0, to: 40, lamps: "G" }, { from: 40, to: 57, lamps: "GA" }, { from: 57, to: 60, lamps: "Y" }, { from: 60, to: 100, lamps: "R" }],
  "375": [{ from: 0, to: 60, lamps: "R" }, { from: 60, to: 97, lamps: "RA" }, { from: 97, to: 100, lamps: "RY" }],
};
// Route signals: stage name and whether the course turns left there (arrow needed).
const ROUTE_LIGHTS: Record<string, { name: string; left: boolean }> = {
  "142": { name: "F1", left: false },
  "143": { name: "F2", left: false },
  "305": { name: "F3 좌회전", left: true },
  "375": { name: "F4 좌회전", left: true },
  "322": { name: "F5", left: false },
  "318": { name: "Q2", left: false },
  "144": { name: "Q4", left: false },
};
const LAMP_TEXT: Record<string, string> = { R: "적", Y: "황", G: "녹", A: "화살표" };
const LAMP_BITS: Record<string, number> = { R: 1, Y: 2, G: 4, A: 8 };
const FRONT_M = 1.98;

function lightNumber(id: string): string {
  const match = /(\d+)$/.exec(id);
  return match?.[1] ?? id;
}

function windowAt(program: Window[], t: number): Window | undefined {
  return program.find((item) => t >= item.from && t < item.to);
}

function permitted(lamps: string, turn: { left: boolean }): boolean {
  return turn.left ? lamps.includes("A") : lamps.includes("G");
}

function lampText(lamps: string): string {
  return lamps.split("").map((lamp) => LAMP_TEXT[lamp] ?? lamp).join("+");
}

function lampsFromBits(bits: number | undefined): string {
  if (bits == undefined || bits === 0) {
    return "";
  }
  return Object.entries(LAMP_BITS).filter(([, bit]) => (bits & bit) !== 0).map(([lamp]) => lamp).join("");
}

// Seconds until the permission of this light changes (to permitted, or to not permitted).
function untilChange(program: Window[], t: number, turn: { left: boolean }): number {
  const now = permitted(windowAt(program, t)?.lamps ?? "", turn);
  for (let step = 0.1; step <= PERIOD_S; step += 0.1) {
    const later = (t + step) % PERIOD_S;
    if (permitted(windowAt(program, later)?.lamps ?? "", turn) !== now) {
      return step;
    }
  }
  return Number.POSITIVE_INFINITY;
}

export function SignalClock({ model, offsetSec }: { model: MonitorModel; offsetSec: number }): ReactElement {
  const now = model.currentTime;
  if (now == undefined) {
    return <div className="signal-clock">신호 주기: 시각 대기</div>;
  }
  const cycle = ((now - offsetSec) % PERIOD_S + PERIOD_S) % PERIOD_S;
  const lightIds = arrayField(model.route, "next_traffic_light_ids").filter((id): id is string => typeof id === "string");
  const target = lightIds.map(lightNumber).find((id) => ROUTE_LIGHTS[id] != undefined);
  const rearToLine = numberField(model.route, "distance_to_stopline");
  const speed = Math.abs(numberField(model.vehicle, "speed_mps") ?? numberField(model.drive, "speed_mps") ?? 0);

  let detail: ReactElement | undefined;
  if (target != undefined) {
    const info = ROUTE_LIGHTS[target]!;
    const program = PROGRAMS[target]!;
    const lamps = windowAt(program, cycle)?.lamps ?? "";
    const ok = permitted(lamps, info);
    const change = untilChange(program, cycle, info);
    const frontGap = rearToLine != undefined && rearToLine < 200 ? Math.max(0, rearToLine - FRONT_M) : undefined;
    const eta = frontGap != undefined && speed > 0.5 ? frontGap / speed : undefined;
    const arrival = eta == undefined ? undefined : (cycle + eta) % PERIOD_S;
    const arrivalOk = arrival == undefined ? undefined : permitted(windowAt(program, arrival)?.lamps ?? "", info);
    const estimate = arrayField(model.signalPhases, "phases")
      .map(asObject)
      .find((item): item is RosObject => item != undefined && lightNumber(textField(item, "light_id")) === target);
    const estimateCycle = numberField(estimate, "cycle_time_sec");
    const drift = estimateCycle == undefined ? undefined : ((estimateCycle - cycle + 150) % PERIOD_S) - 50;
    detail = (
      <div className="signal-detail">
        <div>
          <span>대상 {info.name} ({target})</span>
          <b className={ok ? "status-good" : "status-stop"}>
            {lampText(lamps)} · {ok ? "진행 허용" : "정지"} · {Number.isFinite(change) ? `${change.toFixed(1)} s 뒤 ${ok ? "끝남" : "허용"}` : ""}
          </b>
        </div>
        <div>
          <span>도착 예상</span>
          <b className={arrivalOk == undefined ? "" : arrivalOk ? "status-good" : "status-stop"}>
            {eta == undefined || arrival == undefined ? "정차 또는 거리 없음" : `${eta.toFixed(1)} s 뒤 · 주기 ${arrival.toFixed(1)} s · ${arrivalOk === true ? "허용" : "정지"}`}
          </b>
        </div>
        <div>
          <span>현시 추정기</span>
          <b>
            {estimate == undefined
              ? "토픽 없음"
              : `${boolField(estimate, "aligned") === true ? "정렬" : "미정렬"} · ${lampText(lampsFromBits(numberField(estimate, "predicted_lamps"))) || "--"} · 시간표 대비 ${drift == undefined ? "--" : `${drift >= 0 ? "+" : ""}${drift.toFixed(1)} s`}`}
          </b>
        </div>
      </div>
    );
  }
  const routeRow = Object.entries(ROUTE_LIGHTS)
    .filter(([id]) => (target == undefined ? true : id !== target))
    .map(([id, info]) => {
      const lamps = windowAt(PROGRAMS[id]!, cycle)?.lamps ?? "";
      return { id, name: info.name.split(" ")[0], ok: permitted(lamps, info) };
    });

  return (
    <div className="signal-clock">
      <div className="signal-head">
        <span>신호 주기 (UTC {offsetSec >= 0 ? "+" : ""}{offsetSec} s)</span>
        <strong>{cycle.toFixed(1)} / {PERIOD_S} s</strong>
      </div>
      <i className="signal-bar"><b style={{ width: `${cycle}%` }} /></i>
      {detail}
      <div className="signal-row">
        {routeRow.map((item) => (
          <span key={item.id} className={item.ok ? "status-good" : "status-stop"}>{item.name}</span>
        ))}
      </div>
    </div>
  );
}
