import React, { useEffect, useMemo, useState } from "react";
import {
  jog,
  RobotState,
  setTarget,
  runPickPlace,
  setBarHeight,
} from "../api";

// Joint limits (degrees) — updated to real-world limits
const LIMITS: Array<[number, number]> = [
  [-360, 360],
  [-360, 360],
  [-360, 360],
  [-360, 360],
  [-360, 360],
  [-360, 360],
];

// A common UR-style “ready/home” pose for a 6-axis arm (degrees).
// Chosen here for a clear elbow-bent visual posture.
// Rotate J1 so the arm is to the side (L-shape) and avoids the centerline bar.
const HOME_DEG = [90, -90, 90, -90, 0, 0];

function clamp(v: number, lo: number, hi: number) {
  return Math.max(lo, Math.min(hi, v));
}

export default function ControlPad({
  state,
  onUpdateState,
}: {
  state: RobotState | null;
  onUpdateState: (s: RobotState) => void;
}) {
  const current = useMemo(
    () => state?.joints_deg ?? [0, 0, 0, 0, 0, 0],
    [state]
  );

  const [targets, setTargets] = useState<number[]>(current);
  const [moveSeconds, setMoveSeconds] = useState<number>(2.0);

  const [barHeight, setBarHeightLocal] = useState<number>(() => (typeof state?.bar_height === "number" ? state!.bar_height! : 0.6));

  // Fixed scenario: backend owns object + pick/place positions + obstacle.

  useEffect(() => {
    setTargets(current);
  }, [current.join(",")]); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    if (typeof state?.bar_height === "number") setBarHeightLocal(state.bar_height);
  }, [state?.bar_height]);

  async function sendTarget() {
    const clamped = targets.map((v, i) => clamp(v, LIMITS[i][0], LIMITS[i][1]));
    try {
      const s = await setTarget(clamped, moveSeconds);
      onUpdateState(s);
    } catch (err: any) {
      console.error(err?.message || "Planning failed; target not executed.");
      alert(err?.message || "Planning failed; target not executed.");
    }
  }

  async function doJog(i: number, delta: number) {
    try {
      const s = await jog(i, delta, 0.5);
      onUpdateState(s);
    } catch (err: any) {
      console.error(err?.message || "Jog failed (planning)");
      alert(err?.message || "Jog failed (planning)");
    }
  }

  async function doRun() {
    try {
      const s = await runPickPlace(moveSeconds);
      onUpdateState(s);
    } catch (err: any) {
      console.error(err?.message || "Task failed");
      alert(err?.message || "Task failed");
    }
  }

  async function updateBarHeight(h: number) {
    setBarHeightLocal(h);
    try {
      await setBarHeight(h);
    } catch (err: any) {
      console.error(err?.message || "Failed to set bar height");
      alert(err?.message || "Failed to set bar height");
    }
  }

  return (
    <div className="card">
      {/* --- Joints control --- */}
      <div style={{ display: "flex", justifyContent: "space-between", gap: 10 }}>
        <div>
          <strong>Control Pad</strong>
          <div className="small">Set joint targets or jog (+/-).</div>
        </div>
        <div style={{ textAlign: "right" }}>
          <div className="small">Move time (s)</div>
          <input
            type="number"
            value={moveSeconds}
            min={0.1}
            max={20}
            step={0.1}
            onChange={(e) => setMoveSeconds(parseFloat(e.target.value))}
            style={{
              width: 90,
              padding: 6,
              borderRadius: 10,
              border: "1px solid #ddd",
            }}
          />
        </div>
      </div>

      {targets.map((v, i) => (
        <div key={i} className="row">
          <div>
            <div className="small">
              J{i + 1} ({LIMITS[i][0]}..{LIMITS[i][1]}°) target: {v.toFixed(1)}°
            </div>
            <input
              type="range"
              min={LIMITS[i][0]}
              max={LIMITS[i][1]}
              step={0.5}
              value={v}
              onChange={(e) => {
                const nv = parseFloat(e.target.value);
                setTargets((t) => {
                  const copy = [...t];
                  copy[i] = nv;
                  return copy;
                });
              }}
            />
          </div>

          <button className="btn" onClick={() => doJog(i, -5)} title="Jog -5°">
            −5°
          </button>
          <button className="btn" onClick={() => doJog(i, +5)} title="Jog +5°">
            +5°
          </button>
        </div>
      ))}

      <div style={{ display: "flex", gap: 8, marginTop: 8 }}>
        <button className="btn" onClick={sendTarget} style={{ flex: 1 }}>
          Move to targets
        </button>
        <button
          className="btn"
          onClick={() => setTargets(HOME_DEG)}
          style={{ flex: 1 }}
        >
          Home pose
        </button>
      </div>

      <div className="card" style={{ marginTop: 10 }}>
        <div style={{ display: "flex", justifyContent: "space-between", gap: 10 }}>
          <div>
            <strong>Pick & Place (fixed tables)</strong>
            <div className="small">
              Object: {state?.object ? "set" : "not set"} · {state?.grasped ? "grasped" : "released"}
            </div>
          </div>
        </div>

        <div style={{ marginTop: 10 }}>
          <div className="small">Center bar height (m)</div>
          <div style={{ display: "flex", gap: 8, alignItems: "center" }}>
            <input
              type="range"
              min={0.1}
              max={0.9}
              step={0.01}
              value={barHeight}
              onChange={(e) => updateBarHeight(parseFloat(e.target.value))}
              style={{ flex: 1 }}
            />
            <input
              type="number"
              min={0.1}
              max={0.9}
              step={0.01}
              value={barHeight}
              onChange={(e) => updateBarHeight(parseFloat(e.target.value))}
              style={{ width: 90, padding: 6, borderRadius: 10, border: "1px solid #ddd" }}
            />
          </div>
        </div>

        <div style={{ display: "flex", gap: 8, marginTop: 8 }}>
          <button className="btn" onClick={doRun} style={{ flex: 1 }} disabled={!!state?.moving}>
            Run pick & place
          </button>
        </div>
      </div>

      {/* collision messages removed */}

      {/* Pick & Place and Obstacles editor removed */}
    </div>
  );
}
