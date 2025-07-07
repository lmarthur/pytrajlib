import React, { useEffect, useState } from "react";
import LaunchPointBar from "@/app/components/sidebarComponents/LaunchpointBar";
import AimPointBar from "@/app/components/sidebarComponents/AimpointBar";
import Button from "@/app/components/Button";
import { useMapContext } from "@/app/page";
import runParamsFile from "../../default.json";
import createTrajlib from "../../trajlib.js";
import { Varela_Round } from "next/font/google";
import { sphere2cart, transformToEarthCoords, calculateCEP } from "@/app/utils/utils";

const varelaRound = Varela_Round({
  subsets: ['latin'],
  weight: ["400"],
  variable: '--font-varela',
});

const extractImpactData = (impactDataStr, launchpoint) => {
  /**
   * Extract impact data from the string and transform it to Earth coordinates.
   */
  const impactData = impactDataStr
    .trim()
    .split("\n")
    .map((line) => line.split(",").map((num) => parseFloat(num.trim())));
  const cep = calculateCEP(impactData, launchpoint);
  // The first row contains the aimpoint coordinates
  // Shifting removes it so we only iterate over the impact data
  impactData.shift();

  const strikepoints = [];
  for (const row of impactData) {
    const x = row[1];
    const y = row[2];
    const z = row[3];
    const [latitude, longitude] = transformToEarthCoords(x, y, z, launchpoint);
    strikepoints.push([
      (latitude * 180) / Math.PI,
      (longitude * 180) / Math.PI,
    ]);
  }
  return [strikepoints, cep];
}

const extractTrajectoryData = (trajectoryDataStr, launchpoint) => {
  /**
   * Extract trajectory data from the string and transform it to Earth coordinates.
   */
  const trajectoryData = trajectoryDataStr
    .trim()
    .split("\n")
    .map((line) => line.split(",").map((num) => parseFloat(num.trim())));
  const trajectory = [];
  for (const row of trajectoryData.slice(1)) {
    const t = row[0];
    const x = row[2];
    const y = row[3];
    const z = row[4];
    const [latitude, longitude] = transformToEarthCoords(x, y, z, launchpoint);
    trajectory.push([
      t,
      (latitude * 180) / Math.PI,
      (longitude * 180) / Math.PI,
    ]);
  }
  return trajectory;
};

const runSimulation = async (trajlib, runParams, launchpoint) => {
  const types = Object.values(runParams).map((p) => typeof p);
  console.log("runParams:");
  console.log(runParams);

  const data = await trajlib.ccall(
    "mc_run_wrapper", // C function name
    "string", // C return type
    types, // argument types
    [
      ...Object.values(runParams),
    ]
  );
  // First part of data is the impact data, second part is the trajectory data
  const [impactDataStr, trajectoryDataStr] = data.split("\nTrajectory Data:\n");
  const [strikepoints, cep] = extractImpactData(impactDataStr, launchpoint);
  const trajectoryData = extractTrajectoryData(trajectoryDataStr, launchpoint);
  return {
    strikepoints: strikepoints,
    trajectoryData: trajectoryData,
    cep: cep,
  };
};

export default function Sidebar() {
  const [simRunning, setSimRunning] = useState(false);
  const [runParams, setRunParams] = useState(runParamsFile);
  const {
    launchpoint,
    aimpoint,
    setStrikepoints,
    setTrajectoryData,
    setCEP,
  } = useMapContext();

  useEffect(() => {
    if (aimpoint.lat != null && aimpoint.lon != null) {
      const aim_lon = (aimpoint.lon * Math.PI) / 180;
      const aim_lat = (aimpoint.lat * Math.PI) / 180;
      const [x, y, z] = sphere2cart([6371e3, aim_lon, aim_lat]);
      setRunParams((prev) => ({
        ...prev,
        x_aim: x,
        y_aim: y,
        z_aim: z,
      }));
    }
    if (launchpoint.lat != null && launchpoint.lon != null) {
      const launch_lon = (launchpoint.lon * Math.PI) / 180;
      const launch_lat = (launchpoint.lat * Math.PI) / 180;
      const [x, y, z] = sphere2cart([6371e3, launch_lon, launch_lat]);
      setRunParams((prev) => ({
        ...prev,
        x_launch: x,
        y_launch: y,
        z_launch: z,
      }));
    }
  }, [aimpoint.lat, aimpoint.lon, launchpoint.lat, launchpoint.lon]);

  const simReady =
    launchpoint.lat != null &&
    launchpoint.lon != null &&
    aimpoint.lat != null &&
    aimpoint.lon != null;

  return (
    <div className="w-64 h-full bg-gradient-to-b from-gray-800 to-cyan-990 text-white p-4">
      <div className={`text-2xl text-center mb-8 ${varelaRound.className}`}>
        TRAJMAP
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
              const { strikepoints, trajectoryData, cep } = await runSimulation(
                Module,
                runParams,
                launchpoint,
              );
              console.log("receiving strikepoints:", strikepoints);
              setStrikepoints(strikepoints);
              setTrajectoryData(trajectoryData);
              setSimRunning(false);
              setCEP(cep);
            });
          }
        }}
      />
    </div>
  );
}
