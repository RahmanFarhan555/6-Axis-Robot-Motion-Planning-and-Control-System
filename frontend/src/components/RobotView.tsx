import React, { useEffect, useMemo, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid } from "@react-three/drei";
import * as THREE from "three";

import { ColladaLoader } from "three/examples/jsm/loaders/ColladaLoader.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import URDFLoader from "urdf-loader";

import type { ObjectBox } from "../api";
import { API_BASE } from "../api";

const JOINTS = [
  "shoulder_pan_joint",
  "shoulder_lift_joint",
  "elbow_joint",
  "wrist_1_joint",
  "wrist_2_joint",
  "wrist_3_joint",
];

const deg2rad = (d: number) => (d * Math.PI) / 180;

function UR5Robot({ jointsDeg, liftZ }: { jointsDeg: number[]; liftZ?: number }) {
  const [robot, setRobot] = useState<any>(null);
  const [jointSigns, setJointSigns] = useState<number[]>(() => JOINTS.map(() => 1));

  useEffect(() => {
    const loader = new URDFLoader();
    loader.packages = { ur_description: "/ur5" };

    loader.loadMeshCb = (
      path: string,
      manager: THREE.LoadingManager,
      done: (obj: any) => void
    ) => {
      const ext = path.split(".").pop()?.toLowerCase();

      if (ext === "dae") {
        const l = new ColladaLoader(manager);
        l.load(
          path,
          (dae) => done(dae.scene),
          undefined,
          (err) => {
            console.error("DAE load failed:", path, err);
            done(null);
          }
        );
        return;
      }

      if (ext === "stl") {
        const l = new STLLoader(manager);
        l.load(
          path,
          (geom) => {
            const mesh = new THREE.Mesh(geom, new THREE.MeshNormalMaterial());
            done(mesh);
          },
          undefined,
          (err) => {
            console.error("STL load failed:", path, err);
            done(null);
          }
        );
        return;
      }

      console.warn("Unknown mesh format:", path);
      done(null);
    };

    loader.load(
      "/ur5/ur5.urdf",
      (urdfRobot) => {
        // apply explicit lift if provided so the robot appears elevated
        if (typeof liftZ === "number") {
          urdfRobot.position.set(0, 0, liftZ);
        }
        setRobot(urdfRobot);
      },
      undefined,
      (err) => console.error("URDF load failed:", err)
    );
  }, []);

  const jointsRad = useMemo(() => jointsDeg.map(deg2rad), [jointsDeg]);

  useEffect(() => {
    if (!robot) return;
    JOINTS.forEach((name, i) => {
      const sign = jointSigns[i] || 1;
      const val = (jointsRad[i] || 0) * sign;
      robot.joints?.[name]?.setJointValue(val);
    });
  }, [robot, jointsRad, jointSigns]);

  // Fetch joint axes from backend and determine sign corrections so
  // the frontend joint positive rotation follows the standard UR5 axes.
  useEffect(() => {
    let mounted = true;
    async function fetchAxes() {
      try {
        const resp = await fetch(`${API_BASE}/api/debug/joint_axes`);
        if (!resp.ok) return;
        const data = await resp.json();
        const axesInfo: Array<{ index: number; name: string; axis: number[] }> = data.axes || [];

        const desiredAxes: number[][] = [
          [0, 0, 1], // J1
          [0, 1, 0], // J2
          [0, 1, 0], // J3
          [0, 1, 0], // J4
          [1, 0, 0], // J5
          [0, 1, 0], // J6
        ];

        const signs = JOINTS.map((jn, i) => {
          const info = axesInfo.find((a) => a.name === jn);
          if (!info || !info.axis || info.axis.length < 3) return 1;
          const ax = info.axis;
          const normA = Math.hypot(ax[0], ax[1], ax[2]);
          const normD = Math.hypot(desiredAxes[i][0], desiredAxes[i][1], desiredAxes[i][2]);
          if (normA < 1e-6 || normD < 1e-6) return 1;
          const dot = (ax[0] * desiredAxes[i][0] + ax[1] * desiredAxes[i][1] + ax[2] * desiredAxes[i][2]) / (normA * normD);
          return dot >= 0 ? 1 : -1;
        });

        // Allow manual overrides for known bad axes (e.g. J2 going into ground).
        // Set any overrides here as { jointName: sign }.
        const OVERRIDES: Record<string, number> = {
          // Example: force shoulder_lift_joint (J2) to invert if needed:
          // "shoulder_lift_joint": -1,
        };
        const finalSigns = signs.map((s, i) => (OVERRIDES[JOINTS[i]] ? OVERRIDES[JOINTS[i]] : s));

        if (mounted) {
          setJointSigns(finalSigns);
          console.info("Joint axis signs computed:", finalSigns);
        }
      } catch (err) {
        console.warn("Could not fetch joint axes:", err);
      }
    }
    fetchAxes();
    return () => {
      mounted = false;
    };
  }, [robot]);

  if (!robot) return null;

  // NOTE: no rotation or extra translation here; rotation is applied in parent group
  return <primitive object={robot} />;
}

export default function RobotView({
  jointsDeg,
  object,
  barHeight,
}: {
  jointsDeg: number[];
  object?: ObjectBox | null;
  barHeight?: number | null;
}) {
  const LIFT_Z = 0.5;

  // Fixed two-table scenario (world frame)
  const TABLE1 = useMemo(
    () => ({ x: 0.35, y: 0.35, topZ: 0.5, sx: 0.45, sy: 0.6, sz: 0.04 }),
    []
  );
  const TABLE2 = useMemo(
    () => ({ x: -0.35, y: 0.35, topZ: 0.5, sx: 0.45, sy: 0.6, sz: 0.04 }),
    []
  );

  const BAR = useMemo(
    () => ({ x: 0.0, y: 0.35, sx: 0.04, sy: 0.12 }),
    []
  );
  // support cylinder geometry for the gap between ground and robot base
  const supportGeom = useMemo(() => {
    // CylinderGeometry defaults height along the Y axis; keep as-is so
    // the mesh appears vertical in world coordinates.
    const g = new THREE.CylinderGeometry(0.05, 0.05, LIFT_Z, 32);
    return g;
  }, [LIFT_Z]);

  return (
    <Canvas camera={{ position: [1.6, 1.2, 1.8], fov: 50 }}>
      <ambientLight intensity={1.0} />
      <directionalLight position={[5, 5, 5]} intensity={1.2} />
      <Grid args={[10, 10]} />
      <OrbitControls />

      {/* rotate the scene to match URDF orientation; UR5Robot will apply liftZ */}
      <group rotation={[-Math.PI / 2, 0, 0]}>
        {/* support cylinder placed in same rotated frame so it aligns with robot base.
            Counter-rotate the mesh so its axis is vertical in world coordinates. */}
        <mesh position={[0, 0, LIFT_Z / 2]} rotation={[Math.PI / 2, 0, 0]} geometry={supportGeom}>
          <meshStandardMaterial color={0x333333} metalness={0.2} roughness={0.6} />
        </mesh>

        {/* fixed tables */}
        <mesh position={[TABLE1.x, TABLE1.y, TABLE1.topZ - TABLE1.sz / 2]}>
          <boxGeometry args={[TABLE1.sx, TABLE1.sy, TABLE1.sz]} />
          <meshStandardMaterial color={0x666666} metalness={0.05} roughness={0.85} />
        </mesh>

        <mesh position={[TABLE2.x, TABLE2.y, TABLE2.topZ - TABLE2.sz / 2]}>
          <boxGeometry args={[TABLE2.sx, TABLE2.sy, TABLE2.sz]} />
          <meshStandardMaterial color={0x666666} metalness={0.05} roughness={0.85} />
        </mesh>

        {/* vertical bar obstacle between the tables */}
        {typeof barHeight === "number" && barHeight > 0 && (
          <mesh position={[BAR.x, BAR.y, barHeight / 2]}>
            <boxGeometry args={[BAR.sx, BAR.sy, barHeight]} />
            <meshStandardMaterial color={0xaa3333} metalness={0.05} roughness={0.85} />
          </mesh>
        )}

        {object && (
          <mesh position={[object.x, object.y, object.z]}>
            <boxGeometry args={[object.sx, object.sy, object.sz]} />
            <meshStandardMaterial color={0x33aa33} emissive={0x001a00} transparent opacity={0.65} />
          </mesh>
        )}

        <UR5Robot jointsDeg={jointsDeg} liftZ={LIFT_Z} />
      </group>
    </Canvas>
  );
}
