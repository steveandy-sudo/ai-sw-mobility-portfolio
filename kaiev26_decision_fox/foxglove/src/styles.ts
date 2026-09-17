export const styles = `
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin: 0; }
  .kai-fox {
    --bg: #0a0e10;
    --panel: #111719;
    --panel-2: #151c1f;
    --line: #354044;
    --muted: #8fa1a8;
    --text: #eef5f6;
    --cyan: #20d7eb;
    --amber: #f4b84a;
    --coral: #ff6266;
    --green: #43db91;
    width: 100%;
    height: 100%;
    min-width: 0;
    min-height: 0;
    overflow: auto;
    background: var(--bg);
    color: var(--text);
    font-family: Inter, "Noto Sans KR", system-ui, sans-serif;
    letter-spacing: 0;
  }
  .monitor-shell {
    display: grid;
    grid-template-rows: 54px minmax(0, 1fr);
    width: 100%;
    height: 100%;
    min-width: 0;
    min-height: 700px;
  }
  .monitor-header {
    display: grid;
    grid-template-columns: minmax(190px, 1fr) auto auto auto;
    align-items: center;
    gap: 14px;
    padding: 0 18px;
    border-bottom: 1px solid var(--line);
    background: #0d1214;
  }
  .brand { display: flex; align-items: baseline; gap: 10px; min-width: 0; overflow: hidden; }
  .brand strong { font-size: 14px; color: var(--text); white-space: nowrap; }
  .brand span { min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  .brand span, .header-stat span { font-size: 10px; color: var(--muted); }
  .monitor-header > *, .monitor-grid > *, .summary-panel > * { min-width: 0; }
  .header-stat { display: grid; gap: 2px; min-width: 0; text-align: right; }
  .header-stat strong { font-size: 12px; white-space: nowrap; }
  .live-pill { color: var(--green); }
  .replay-pill { color: var(--amber); }
  .monitor-grid { display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1.08fr); min-width: 0; min-height: 0; }
  .left-stack { display: grid; grid-template-rows: minmax(260px, 44fr) minmax(360px, 56fr); min-width: 0; min-height: 0; border-right: 1px solid var(--line); }
  .right-stack { display: grid; grid-template-rows: minmax(210px, 30fr) minmax(210px, 26fr) minmax(145px, 22fr) minmax(145px, 22fr); min-width: 0; min-height: 0; }
  .map-grid, .camera-grid { display: grid; grid-template-columns: 1fr 1fr; min-height: 0; }
  .map-panel, .health-panel, .camera-panel, .summary-panel, .plot-panel, .fsm-section { position: relative; min-width: 0; min-height: 0; border-bottom: 1px solid var(--line); }
  .map-grid > * + *, .camera-panel + .camera-panel { border-left: 1px solid var(--line); }
  .panel-title { position: absolute; z-index: 3; top: 10px; left: 12px; font-size: 10px; font-weight: 800; color: #a9c4cb; text-transform: uppercase; }
  .panel-subtitle { position: absolute; z-index: 3; top: 27px; left: 12px; font-size: 9px; color: var(--muted); }
  .map-svg { display: block; width: 100%; height: 100%; background: #0c1215; }
  .map-legend { position: absolute; z-index: 3; left: 10px; bottom: 8px; display: flex; flex-wrap: wrap; gap: 10px; padding: 6px 8px; background: rgba(8, 13, 15, 0.86); border: 1px solid #303b3f; font-size: 9px; }
  .legend-line { display: inline-flex; align-items: center; gap: 5px; color: #c6d1d4; }
  .legend-line i { display: block; width: 18px; height: 3px; background: currentColor; }
  .legend-point { display: inline-flex; align-items: center; gap: 5px; color: #c6d1d4; }
  .legend-dot { display: block; width: 7px; height: 7px; border-radius: 50%; }
  .waypoint-dot { background: #f4f7f7; }
  .stop-dot { background: var(--coral); border: 1px solid #fff4f4; }
  .health-panel { display: grid; grid-template-rows: auto minmax(0, 1fr) auto; gap: 9px; padding: 12px; overflow: hidden; background: #0c1215; }
  .health-heading { display: flex; align-items: center; justify-content: space-between; gap: 10px; min-width: 0; }
  .health-heading > div:first-child { display: grid; min-width: 0; gap: 4px; }
  .health-heading strong { overflow: hidden; color: #dce7e9; font-size: 11px; text-overflow: ellipsis; white-space: nowrap; }
  .health-overall { display: inline-flex; align-items: center; flex: 0 0 auto; gap: 6px; font-size: 10px; font-weight: 800; }
  .health-overall i, .health-row i { display: block; flex: 0 0 auto; width: 9px; height: 9px; border-radius: 50%; background: #7c8a8f; box-shadow: 0 0 0 2px rgba(124, 138, 143, 0.14); }
  .health-grid { display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1fr); align-content: center; gap: 6px 10px; min-height: 0; }
  .health-row { display: grid; grid-template-columns: 10px minmax(0, 1fr) auto; align-items: center; gap: 6px; min-width: 0; padding: 5px 6px; border: 1px solid #293438; background: #11181a; }
  .health-row span { overflow: hidden; color: #c5d0d3; font-size: 9px; text-overflow: ellipsis; white-space: nowrap; }
  .health-row b { font-size: 8px; }
  .health-level-0 { color: var(--green); }
  .health-level-0 i { background: var(--green); box-shadow: 0 0 0 2px rgba(67, 219, 145, 0.15), 0 0 10px rgba(67, 219, 145, 0.35); }
  .health-level-1 { color: var(--amber); }
  .health-level-1 i { background: var(--amber); box-shadow: 0 0 0 2px rgba(244, 184, 74, 0.15); }
  .health-level-2, .health-level-3 { color: var(--coral); }
  .health-level-2 i, .health-level-3 i { background: var(--coral); box-shadow: 0 0 0 2px rgba(255, 98, 102, 0.16), 0 0 10px rgba(255, 98, 102, 0.3); }
  .health-foot { color: #687b81; font-size: 8px; }
  .camera-panel { background: #070a0b; overflow: hidden; }
  .camera-media { position: absolute; inset: 0; display: grid; place-items: center; }
  .camera-media img { position: absolute; inset: 0; width: 100%; height: 100%; object-fit: contain; }
  .camera-empty { color: #64757b; font-size: 12px; text-align: center; line-height: 1.8; }
  .fsm-section { padding: 12px; background: #0c1113; }
  .section-heading { display: flex; align-items: baseline; justify-content: space-between; margin-bottom: 9px; }
  .section-heading h2 { margin: 0; font-size: 12px; }
  .section-heading span { font-size: 9px; color: var(--muted); }
  .fsm-grid { height: calc(100% - 28px); display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1fr); grid-template-rows: repeat(3, minmax(76px, 1fr)); gap: 8px; }
  .fsm-card { position: relative; min-width: 0; overflow: hidden; padding: 9px 11px; border: 1px solid #303b3f; border-left: 5px solid #526067; background: var(--panel); border-radius: 4px; }
  .fsm-card.active { border-color: var(--cyan); border-left-color: var(--cyan); background: #0e2428; box-shadow: inset 0 0 24px rgba(32, 215, 235, 0.08); }
  .fsm-card h3 { margin: 0 0 5px; font-size: 13px; color: #b6c5ca; }
  .fsm-card strong { display: block; overflow: hidden; font-size: 18px; color: #60747b; line-height: 1.05; text-overflow: ellipsis; white-space: nowrap; }
  .fsm-card.active strong { color: var(--cyan); }
  .fsm-card p { margin: 6px 0 0; font-size: 10px; color: #8ba0a7; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  .fsm-card.active p { color: #e0f5f7; }
  .fsm-index { position: absolute; top: 9px; right: 10px; font-size: 9px; color: #607078; }
  .fsm-status { position: absolute; right: 10px; bottom: 8px; font-size: 8px; color: #64757b; }
  .fsm-card.active .fsm-status { color: var(--cyan); }
  .summary-panel { display: grid; grid-template-columns: minmax(0, 1.08fr) minmax(280px, 0.92fr); background: var(--panel); }
  .selected-state { display: flex; flex-direction: column; justify-content: center; padding: 18px 20px; border-right: 1px solid var(--line); min-width: 0; }
  .eyebrow { font-size: 9px; color: #83a0a8; text-transform: uppercase; }
  .selected-state strong { margin: 7px 0; color: var(--cyan); font-size: 23px; line-height: 1.1; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  .selected-state p { margin: 0; color: #d7e3e5; font-size: 11px; }
  .hold-timer { display: grid; gap: 5px; margin-top: 9px; }
  .hold-timer > div { display: flex; align-items: baseline; justify-content: space-between; gap: 8px; }
  .hold-timer span { color: var(--muted); font-size: 9px; }
  .hold-timer strong { margin: 0; color: var(--amber); font-size: 13px; }
  .hold-timer > i { display: block; width: 100%; height: 5px; overflow: hidden; background: #303a3e; }
  .hold-timer > i > b { display: block; height: 100%; background: var(--amber); }
  .command-chain { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; margin-top: 12px; padding-top: 9px; border-top: 1px solid #2b3438; }
  .command-chain div { display: grid; min-width: 0; gap: 3px; }
  .command-chain span { color: var(--muted); font-size: 8px; }
  .command-chain b { overflow: hidden; color: #dce7e9; font-size: 9px; font-weight: 600; text-overflow: ellipsis; white-space: nowrap; }
  .motion-values { display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1fr); gap: 7px 12px; align-content: center; min-width: 0; padding: 12px 14px; }
  .motion-value { display: grid; min-width: 0; gap: 4px; }
  .motion-value span { font-size: 9px; color: var(--muted); }
  .motion-value strong { overflow: hidden; font-size: 16px; text-overflow: ellipsis; white-space: nowrap; }
  .motion-value.target strong { color: var(--cyan); }
  .motion-value.steer.target strong { color: var(--amber); }
  .route-strip { grid-column: 1 / -1; display: grid; grid-template-columns: repeat(3, 1fr); gap: 7px 10px; padding-top: 8px; border-top: 1px solid #2b3438; }
  .route-strip div { display: grid; gap: 2px; min-width: 0; }
  .route-strip span { font-size: 8px; color: var(--muted); }
  .route-strip strong { font-size: 10px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  .status-good { color: var(--green); }
  .status-stop { color: var(--coral); }
  .plot-panel { padding: 12px 14px 8px; background: #0e1416; }
  .plot-header { display: flex; justify-content: space-between; align-items: baseline; height: 28px; }
  .plot-header h3 { margin: 0; font-size: 11px; }
  .plot-header span { font-size: 9px; color: var(--muted); }
  .plot-svg { display: block; width: 100%; height: calc(100% - 28px); touch-action: none; cursor: crosshair; }
  .plot-label { font-size: 8px; fill: #87999f; }
  .plot-value { font-size: 9px; fill: #dfe9eb; }
  @media (max-width: 1450px) {
    .monitor-header { grid-template-columns: minmax(180px, 1fr) auto auto; }
    .monitor-header .header-stat:nth-child(3) { display: none; }
    .summary-panel { grid-template-columns: minmax(0, 1fr) minmax(250px, 0.9fr); }
    .selected-state strong { font-size: 19px; }
    .fsm-card strong { font-size: 16px; }
  }
  @media (max-width: 1100px) {
    .monitor-shell { height: auto; min-height: 1280px; }
    .monitor-grid { grid-template-columns: minmax(0, 1fr); }
    .left-stack { grid-template-rows: 360px 480px; border-right: 0; }
    .right-stack { grid-template-rows: 280px 220px 210px 210px; }
  }
`;
