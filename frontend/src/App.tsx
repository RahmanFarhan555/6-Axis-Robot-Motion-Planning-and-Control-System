import React, { useEffect, useMemo, useState } from "react";
import { getState, openWS, RobotState } from "./api";
import RobotView from "./components/RobotView";
import ControlPad from "./components/ControlPad";

export default function App() {
  const [state, setState] = useState<RobotState | null>(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    getState().then(setState).catch(console.error);

    const ws = openWS((s) => {
      setConnected(true);
      setState(s);
    });
    ws.onclose = () => setConnected(false);
    return () => ws.close();
  }, []);

  const joints = useMemo(() => state?.joints_deg ?? [0, 0, 0, 0, 0, 0], [state]);

  return (
    <>
      <div className="header">
        <div>
          <strong>6-Axis Robot Control</strong>
        </div>
        <div className="badge">
          WS: {connected ? "connected" : "disconnected"} · {state?.moving ? "moving" : "idle"}
        </div>
      </div>

      <div className="layout">
        <RobotView jointsDeg={joints} object={state?.object ?? null} barHeight={state?.bar_height ?? null} />

        <div className="panel">
          <ControlPad state={state} onUpdateState={setState} />

          <div className="card">
            <div>
              <strong>Current joints (deg)</strong>
            </div>
            <div className="small">
              {joints.map((v, i) => `J${i + 1}: ${v.toFixed(1)}`).join(" · ")}
            </div>
          </div>
        </div>
      </div>
    </>
  );
}
