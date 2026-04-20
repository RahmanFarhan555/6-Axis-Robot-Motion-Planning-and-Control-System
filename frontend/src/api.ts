export type RobotState = {
  joints_deg: number[];
  moving: boolean;
  target_deg?: number[] | null;
  object?: ObjectBox | null;
  grasped?: boolean;
  bar_height?: number | null;
};

export const API_BASE =
  (import.meta as any).env?.VITE_API_BASE ?? "http://localhost:8000";

export type ObjectBox =
  { id: string; x: number; y: number; z: number; sx: number; sy: number; sz: number };

async function parseStateResponse(r: Response, fallbackMessage: string): Promise<RobotState> {
  if (r.ok) return r.json();

  let msg = fallbackMessage;
  try {
    const j = await r.json();
    if (typeof (j as any)?.detail === "string") msg = (j as any).detail;
    else msg = JSON.stringify(j);
  } catch {
    try {
      const text = await r.text();
      if (text) msg = text;
    } catch {
      // ignore
    }
  }
  throw new Error(msg);
}

export async function getState(): Promise<RobotState> {
  const r = await fetch(`${API_BASE}/api/state`);
  if (!r.ok) throw new Error("Failed to fetch state");
  return r.json();
}

export async function setTarget(joints_deg: number[], move_seconds: number) {
  const r = await fetch(`${API_BASE}/api/target`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ joints_deg, move_seconds }),
  });
  if (!r.ok) {
    let msg = "Failed to set target";
    try {
      const j = await r.json();
      if (typeof j?.detail === "string") msg = j.detail;
      else msg = JSON.stringify(j);
    } catch {
      const text = await r.text();
      if (text) msg = text;
    }
    throw new Error(msg);
  }
  return r.json();
}

export async function setObject(object: ObjectBox | null) {
  const r = await fetch(`${API_BASE}/api/object`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ object }),
  });
  if (!r.ok) throw new Error("Failed to set object");
  return r.json();
}

export async function setPickPose() {
  const r = await fetch(`${API_BASE}/api/pick_pose`, {
    method: "POST",
  });
  return parseStateResponse(r, "Failed to set pick pose");
}

export async function setPlacePose() {
  const r = await fetch(`${API_BASE}/api/place_pose`, {
    method: "POST",
  });
  return parseStateResponse(r, "Failed to set place pose");
}

export async function pickXYZ(x: number, y: number, z: number, move_seconds: number) {
  const r = await fetch(`${API_BASE}/api/pick_xyz`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ x, y, z, move_seconds }),
  });
  return parseStateResponse(r, "Failed to pick");
}

export async function placeXYZ(x: number, y: number, z: number, move_seconds: number) {
  const r = await fetch(`${API_BASE}/api/place_xyz`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ x, y, z, move_seconds }),
  });
  return parseStateResponse(r, "Failed to place");
}

export async function runPickPlace(move_seconds: number) {
  const r = await fetch(`${API_BASE}/api/run_pick_place`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ move_seconds }),
  });
  return parseStateResponse(r, "Failed to run pick&place");
}

export async function getBarHeight(): Promise<number> {
  const r = await fetch(`${API_BASE}/api/bar_height`);
  if (!r.ok) throw new Error("Failed to fetch bar height");
  const j = await r.json();
  return typeof j?.height === "number" ? j.height : Number(j?.height);
}

export async function setBarHeight(height: number): Promise<number> {
  const r = await fetch(`${API_BASE}/api/bar_height`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ height }),
  });
  if (!r.ok) {
    let msg = "Failed to set bar height";
    try {
      const j = await r.json();
      if (typeof j?.detail === "string") msg = j.detail;
      else msg = JSON.stringify(j);
    } catch {
      const text = await r.text();
      if (text) msg = text;
    }
    throw new Error(msg);
  }
  const j = await r.json();
  return typeof j?.height === "number" ? j.height : Number(j?.height);
}


export async function pick(move_seconds: number) {
  const r = await fetch(`${API_BASE}/api/pick`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ move_seconds }),
  });
  if (!r.ok) {
    let msg = "Failed to pick";
    try {
      const j = await r.json();
      if (typeof j?.detail === "string") msg = j.detail;
      else msg = JSON.stringify(j);
    } catch {
      const text = await r.text();
      if (text) msg = text;
    }
    throw new Error(msg);
  }
  return r.json();
}

export async function place(move_seconds: number) {
  const r = await fetch(`${API_BASE}/api/place`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ move_seconds }),
  });
  if (!r.ok) {
    let msg = "Failed to place";
    try {
      const j = await r.json();
      if (typeof j?.detail === "string") msg = j.detail;
      else msg = JSON.stringify(j);
    } catch {
      const text = await r.text();
      if (text) msg = text;
    }
    throw new Error(msg);
  }
  return r.json();
}

export async function jog(joint_index: number, delta_deg: number, move_seconds: number) {
  const r = await fetch(`${API_BASE}/api/jog`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ joint_index, delta_deg, move_seconds }),
  });
  if (!r.ok) {
    let msg = "Failed to jog";
    try {
      const j = await r.json();
      if (typeof j?.detail === "string") msg = j.detail;
      else msg = JSON.stringify(j);
    } catch {
      const text = await r.text();
      if (text) msg = text;
    }
    throw new Error(msg);
  }
  return r.json();
}

export function openWS(onState: (s: RobotState) => void) {
  const url = new URL(API_BASE);
  const wsProto = url.protocol === "https:" ? "wss:" : "ws:";
  const wsUrl = `${wsProto}//${url.host}/ws`;
  const ws = new WebSocket(wsUrl);

  ws.onmessage = (evt) => {
    try {
      const data = JSON.parse(evt.data);
      if (data?.joints_deg?.length === 6) onState(data);
    } catch {
      /* ignore */
    }
  };
  ws.onopen = () => ws.send("ping");
  return ws;
}
