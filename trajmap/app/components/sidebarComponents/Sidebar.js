import React, { useEffect, useState } from "react";
import LaunchPointBar from "@/app/components/sidebarComponents/LaunchpointBar";
import AimPointBar from "@/app/components/sidebarComponents/AimpointBar";
import Button from "@/app/components/Button";
import { useMapContext } from "@/app/page";
import runParamsFile from "../../default.json";
import createTrajlib from "../../trajlib.js";

const cartesianToSpherical = (x, y, z) => {
  // cartCoords: [x, y, z]
  // returns [r, longitude, latitude]

  const r = Math.sqrt(x * x + y * y + z * z);
  const longitude = Math.atan2(y, x); // in radians
  const latitude = Math.atan(z / Math.sqrt(x * x + y * y)); // in radians

  return [r, longitude, latitude];
};

const runSimulation = async (trajlib, runParams, aimpoint) => {
  const types = Object.values(runParams).map((p) => typeof p);

  const impactDataStr = await trajlib.ccall(
    "mc_run_wrapper", // C function name
    "string", // C return type
    types, // argument types
    [
      ...Object.values(runParams),
      (aimpoint.lat * Math.PI) / 180,
      (aimpoint.lon * Math.PI) / 180,
    ]
  );
  const impactData = impactDataStr
    .trim()
    .split("\n")
    .map((line) => line.split(",").map((num) => parseFloat(num.trim())));
  const [x_aim, y_aim, z_aim] = impactData.shift();
  console.log("Sim Aimpoint:", x_aim, y_aim, z_aim);
  const aimpoint_lon = (Math.atan2(y_aim, x_aim) * 180) / Math.PI;
  const aimpoint_lat =
    (Math.atan2(z_aim, Math.sqrt(x_aim ** 2 + y_aim ** 2)) * 180) / Math.PI;
  console.log(
    "Sim Aimpoint (spherical) in degrees:",
    aimpoint_lon,
    aimpoint_lat
  );

  const strikepoints = [];
  for (const row of impactData) {
    const x = row[1];
    const y = row[2];
    const z = row[3];
    const [r, longitude, latitude] = cartesianToSpherical(x, y, z);
    strikepoints.push([
      (latitude * 180) / Math.PI,
      (longitude * 180) / Math.PI,
    ]);
  }
  return {
    simAimpoint: { lat: aimpoint_lat, lon: aimpoint_lon },
    strikepoints: strikepoints,
  };
};

export default function Sidebar() {
  const [simRunning, setSimRunning] = useState(false);
  const [runParams, setRunParams] = useState(runParamsFile);
  const {
    launchpoint,
    aimpoint,
    setSimAimpoint,
    setStrikepoints,
  } = useMapContext();

  const simReady =
    launchpoint.lat != null &&
    launchpoint.lon != null &&
    aimpoint.lat != null &&
    aimpoint.lon != null;

  return (
    <div className="w-64 h-full bg-gradient-to-b from-gray-800 to-cyan-990 text-white p-4">
      <div className="text-2xl font-bold text-center mb-8 font-mono">
        TrajMap
      </div>
      <LaunchPointBar />
      <AimPointBar />
      <div className="my-10" />
      <Button
        name={simRunning ? "Simulation Running..." : "Run Simulation"}
        bg={!simReady || simRunning ? "bg-gray-400" : "bg-cyan-500"}
        cursor={
          !simReady || simRunning ? "cursor-not-allowed" : "cursor-pointer"
        }
        onClick={async () => {
          console.log(simReady, simRunning);
          if (simReady) {
            // && !simRunning) {
            console.log("Run Simulation");
            setSimRunning(!simRunning);
            createTrajlib({
              locateFile: (path) => {
                if (path.endsWith(".wasm")) return `/trajlib.wasm`;
                if (path.endsWith(".data")) return `/trajlib.data`;
                return path;
              },
            }).then(async (Module) => {
              console.log("Module created!");
              console.log(Module._test());
              const { simAimpoint, strikepoints } = await runSimulation(
                Module,
                runParams,
                aimpoint
              );
              console.log("receiving strikepoints:", strikepoints);
              setSimAimpoint(simAimpoint);
              setStrikepoints(strikepoints);

              setSimRunning(false);
            });
          }
        }}
      />
    </div>
  );
}
