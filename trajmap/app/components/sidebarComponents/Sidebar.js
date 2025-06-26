import React, { useEffect, useState } from "react";
import LaunchPointBar from "@/app/components/sidebarComponents/LaunchpointBar";
import AimPointBar from "@/app/components/sidebarComponents/AimpointBar";
import Button from "@/app/components/Button";
import { useMapContext } from "@/app/page";
import runParamsFile from "../../default.json";
import createTrajlib from "../../trajlib.js";
import { Varela_Round } from "next/font/google";
import { sphercoordsToCartcoords } from "@/app/utils/utils";

const varelaRound = Varela_Round({ 
  subsets: ['latin'],
  weight:["400"],
  variable: '--font-varela',
});

const cartesianToSpherical = (x, y, z) => {
  // cartCoords: [x, y, z]
  // returns [r, longitude, latitude]

  const r = Math.sqrt(x * x + y * y + z * z);
  const longitude = Math.atan2(y, x); // in radians
  const latitude = Math.atan(z / Math.sqrt(x * x + y * y)); // in radians

  return [r, longitude, latitude];
};

const bearing = (start, end) => {
  /**
   * Calculate the bearing (in radians) from start to end (lat, lon in radians)
   */
  const [launch_lat, launch_lon] = start;
  const [aim_lat, aim_lon] = end;
  const lon_diff = aim_lon - launch_lon;

  // East component
  const east = Math.sin(lon_diff) * Math.cos(aim_lat);

  // North component
  const north =
    Math.cos(launch_lat) * Math.sin(aim_lat) -
    Math.sin(launch_lat) * Math.cos(aim_lat) * Math.cos(lon_diff);

    
  return Math.atan2(north, east);
}

const haversineDistance = (start, end) => {
  /**
   * Calculate the haversine distance between two points (lat, lon in radians)
   */
  const [launch_lat, launch_lon] = start;
  const [aim_lat, aim_lon] = end;
  const a =
    Math.sin((aim_lat - launch_lat) / 2) * Math.sin((aim_lat - launch_lat) / 2) +
    Math.cos(launch_lat) * Math.cos(aim_lat) *
    Math.sin((aim_lon - launch_lon) / 2) * Math.sin((aim_lon - launch_lon) / 2);
  const angular_distance = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  const distance = 6371e3 * angular_distance;
  return distance;
};

const getLocation = (bearing, distance, start) => {
  /**
   * Calculate the end location (lat, lon in radians) given a start location (lat, lon in radians),
   * a bearing (in radians), and a distance (in meters).
   */
  launch_lat = start[0]; // latitude of start point in radians
  launch_long = start[1]; // longitude of start point in radians
  angular_distance = distance / 6371e3;
  const aim_lat = Math.asin( Math.sin(launch_lat)*Math.cos(angular_distance) +
                      Math.cos(launch_lat)*Math.sin(angular_distance)*Math.cos(bearing) );
  const aim_long = launch_long + Math.atan2(Math.sin(bearing)*Math.sin(angular_distance)*Math.cos(launch_lat),
                            Math.cos(angular_distance)-Math.sin(launch_lat)*Math.sin(aim_lat));
  return [aim_lat, aim_long];
};

const extractImpactData = (impactDataStr) => {
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
  return [aimpoint_lat, aimpoint_lon, strikepoints];
}

const extractTrajectoryData = (trajectoryDataStr) => {
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
    const [r, longitude, latitude] = cartesianToSpherical(x, y, z);
    trajectory.push([
      t,
      (latitude * 180) / Math.PI,
      (longitude * 180) / Math.PI,
    ]);
  }
  return trajectory;
};

const runSimulation = async (trajlib, runParams) => {
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
  const [aimpoint_lat, aimpoint_lon, strikepoints] = extractImpactData(impactDataStr);
  const trajectoryData = extractTrajectoryData(trajectoryDataStr);
  return {
    simAimpoint: { lat: aimpoint_lat, lon: aimpoint_lon },
    strikepoints: strikepoints,
    trajectoryData: trajectoryData,
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
    setTrajectoryData,
  } = useMapContext();

  useEffect(() => {
    if (aimpoint.lat != null && aimpoint.lon != null) {
      const aim_lon = (aimpoint.lon * Math.PI) / 180;
      const aim_lat = (aimpoint.lat * Math.PI) / 180;
      const [x, y, z] = sphercoordsToCartcoords([6371e3, aim_lon, aim_lat]);
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
      const [x, y, z] = sphercoordsToCartcoords([6371e3, launch_lon, launch_lat]);
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
              const { simAimpoint, strikepoints, trajectoryData } = await runSimulation(
                Module,
                runParams,
              );
              console.log("receiving strikepoints:", strikepoints);
              setSimAimpoint(simAimpoint);
              setStrikepoints(strikepoints);
              setTrajectoryData(trajectoryData);
              setSimRunning(false);
            });
          }
        }}
      />
    </div>
  );
}
