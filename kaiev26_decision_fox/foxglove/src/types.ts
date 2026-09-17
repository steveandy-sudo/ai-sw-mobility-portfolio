export type RosObject = Record<string, unknown>;

export type Point2 = {
  x: number;
  y: number;
};

export type EgoPose = Point2 & {
  yaw: number;
};

export type ZonePath = {
  name: string;
  color: string;
  points: Point2[];
};

export type TimedValue = {
  time: number;
  value: number;
};

export type CameraFrame = {
  bytes: Uint8Array;
  format: string;
};

export type Histories = {
  speedTarget: TimedValue[];
  speedActual: TimedValue[];
  steerTarget: TimedValue[];
  steerActual: TimedValue[];
};

export type MonitorModel = {
  behavior?: RosObject;
  route?: RosObject;
  readiness?: RosObject;
  readinessTime?: number;
  scene?: RosObject;
  scenario?: RosObject;
  routeReady?: boolean;
  rawCommand?: RosObject;
  planningCommand?: RosObject;
  vehicleCommand?: RosObject;
  targetSpeed?: RosObject;
  vehicle?: RosObject;
  drive?: RosObject;
  steering?: RosObject;
  ego?: EgoPose;
  globalPath: Point2[];
  waypoints: Point2[];
  stopWaypoints: Point2[];
  zonePaths: ZonePath[];
  localPath: Point2[];
  targetPath: Point2[];
  trail: Point2[];
  leftImage?: CameraFrame;
  rightImage?: CameraFrame;
  leftSourceImage?: CameraFrame;
  rightSourceImage?: CameraFrame;
  currentTime?: number;
  startTime?: number;
  endTime?: number;
  isReplay: boolean;
  didSeek: boolean;
  histories: Histories;
};
