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
    setAimpoint,
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

  // Helper for boolean dropdowns
  const BoolDropdown = ({ label, value, onChange }) => (
    <div className="mb-2">
      <label className="block text-xs mb-1">{label}</label>
      <select
        className="w-full p-1 rounded text-black bg-white"
        value={value ? "1" : "0"}
        onChange={e => onChange(e.target.value === "1")}
      >
        <option value="1">Yes</option>
        <option value="0">No</option>
      </select>
    </div>
  );

  // Helper for number/text fill-ins
  const NumberInput = ({ label, value, onChange, step = 1, min, max }) => (
    <div className="mb-2">
      <label className="block text-xs mb-1">{label}</label>
      <input
        className="w-full p-1 rounded text-black bg-white"
        type="number"
        value={value}
        step={step}
        min={min}
        max={max}
        onChange={e => onChange(Number(e.target.value))}
      />
    </div>
  );

  // Helper for string dropdowns
  const StringDropdown = ({ label, value, options, onChange }) => (
    <div className="mb-2">
      <label className="block text-xs mb-1">{label}</label>
      <select
        className="w-full p-1 rounded text-black bg-white"
        value={value}
        onChange={e => onChange(e.target.value)}
      >
        {options.map(opt => (
          <option key={opt.value} value={opt.value}>{opt.label}</option>
        ))}
      </select>
    </div>
  );

  // Booster type options
  const boosterOptions = [
    { value: 0, label: "MMIII" },
    { value: 1, label: "SCUD" },
    { value: 2, label: "SCUD-ER" },
    { value: 3, label: "GBSD" },
    { value: 4, label: "D5" },
    { value: 5, label: "MOCK" },
  ];

  // RV type options
  const rvTypeOptions = [
    { value: 0, label: "Ballistic" },
    { value: 1, label: "Maneuverable" },
  ];

  return (
    <div className="w-80 h-full bg-gradient-to-b from-gray-800 to-cyan-990 text-white p-4 overflow-y-auto max-h-screen">
      <div className={`text-2xl text-center mb-8 ${varelaRound.className}`}>
        TRAJMAP
      </div>
      <LaunchPointBar />
      <AimPointBar />
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

              // Ensure the aimpoint is in the same longitude range as the strikepoints
              // If the strikepoints have negative longitudes, the aimpoint
              // should have a negative longitude as well
              // Same thing if the strikepoints have positive longitudes, the aimpoint
              // should have a positive longitude as well 
              if (strikepoints[0][1] < -180 && aimpoint.lon > 0) {
                const newAimpoint = {
                  lat: aimpoint.lat,
                  lon: aimpoint.lon - 360,
                }
                setAimpoint(newAimpoint);
              }
              else if (strikepoints[0][1] > 180 && aimpoint.lon < 0) {
                const newAimpoint = {
                  lat: aimpoint.lat,
                  lon: aimpoint.lon + 360,
                }
                setAimpoint(newAimpoint);
              }
              setStrikepoints(strikepoints);
              setTrajectoryData(trajectoryData);
              setSimRunning(false);
              setCEP(cep);
            });
          }
        }}
      />
      <div className="my-10" />
      {/* Simulation Parameters Section */}
      <div className="mb-6">
        <div className="font-bold mb-2">Simulation Parameters</div>
        <NumberInput
          label={"Number of Monte Carlo runs"}
          value={runParams.num_runs}
          onChange={v => setRunParams(p => ({ ...p, num_runs: v }))}
          min={1}
        />
        <NumberInput
          label={"Time step (main) in s"}
          value={runParams.time_step_main}
          onChange={v => setRunParams(p => ({ ...p, time_step_main: v }))}
          step={0.01}
          min={0.001}
        />
        <NumberInput
          label={"Time step (reentry) in s"}
          value={runParams.time_step_reentry}
          onChange={v => setRunParams(p => ({ ...p, time_step_reentry: v }))}
          step={0.01}
          min={0.001}
        />
        <BoolDropdown
          label={"Include geoid height uncertainty"}
          value={!!runParams.grav_error}
          onChange={v => setRunParams(p => ({ ...p, grav_error: v ? 1 : 0 }))}
        />
        {/* Atmospheric Model Combined Dropdown */}
        <StringDropdown
          label={"Atmospheric Model"}
          value={(() => {
            if (runParams.atm_model === 0 && runParams.atm_error === 0) return "exp";
            if (runParams.atm_model === 0 && runParams.atm_error === 1) return "expwind";
            if (runParams.atm_model === 1 && runParams.atm_error === 1) return "earthgram";
            return "exp";
          })()}
          options={[
            { value: "exp", label: "Exponential" },
            { value: "expwind", label: "Exponential with Gaussian Wind" },
            { value: "earthgram", label: "EarthGram" },
          ]}
          onChange={v => {
            if (v === "exp") setRunParams(p => ({ ...p, atm_model: 0, atm_error: 0 }));
            else if (v === "expwind") setRunParams(p => ({ ...p, atm_model: 0, atm_error: 1 }));
            else if (v === "earthgram") setRunParams(p => ({ ...p, atm_model: 1, atm_error: 1 }));
          }}
        />
        <BoolDropdown
          label={"Use GNSS navigation"}
          value={!!runParams.gnss_nav}
          onChange={v => setRunParams(p => ({ ...p, gnss_nav: v ? 1 : 0 }))}
        />
        <BoolDropdown
          label={"Perfect INS state measurements"}
          value={!runParams.ins_nav}
          onChange={v => setRunParams(p => ({ ...p, ins_nav: v ? 0 : 1 }))}
        />
        <StringDropdown
          label={"RV maneuverability"}
          value={runParams.rv_maneuv}
          options={[
            { value: 0, label: "None" },
            { value: 1, label: "Proportional (realistic)" },
            { value: 2, label: "Idealized" },
          ]}
          onChange={v => setRunParams(p => ({ ...p, rv_maneuv: Number(v) }))}
        />
        <NumberInput
          label={"Reentry velocity (m/s)"}
          value={runParams.reentry_vel}
          onChange={v => setRunParams(p => ({ ...p, reentry_vel: v }))}
        />
        <NumberInput
          label={"Deflection time (s)"}
          value={runParams.deflection_time}
          onChange={v => setRunParams(p => ({ ...p, deflection_time: v }))}
        />
        <StringDropdown
          label={"Booster type"}
          value={runParams.booster_type}
          options={boosterOptions}
          onChange={v => setRunParams(p => ({ ...p, booster_type: Number(v) }))}
        />
        <StringDropdown
          label={"RV type"}
          value={runParams.rv_type}
          options={rvTypeOptions}
          onChange={v => setRunParams(p => ({ ...p, rv_type: Number(v) }))}
        />
        <NumberInput
          label={"Initial velocity error (m/s)"}
          value={runParams.initial_vel_error}
          onChange={v => setRunParams(p => ({ ...p, initial_vel_error: v }))}
        />
        <NumberInput
          label={"Accelerometer scale stability (ppm)"}
          value={runParams.acc_scale_stability}
          onChange={v => setRunParams(p => ({ ...p, acc_scale_stability: v }))}
        />
        <NumberInput
          label={"Gyro bias stability (rad/s)"}
          value={runParams.gyro_bias_stability}
          onChange={v => setRunParams(p => ({ ...p, gyro_bias_stability: v }))}
        />
        <NumberInput
          label={"Gyro noise (rad/s/sqrt(s))"}
          value={runParams.gyro_noise}
          onChange={v => setRunParams(p => ({ ...p, gyro_noise: v }))}
        />
        <NumberInput
          label={"GNSS noise (m)"}
          value={runParams.gnss_noise}
          onChange={v => setRunParams(p => ({ ...p, gnss_noise: v }))}
        />
        <NumberInput
          label={"Coefficient of lift perturbation"}
          value={runParams.cl_pert}
          onChange={v => setRunParams(p => ({ ...p, cl_pert: v }))}
        />
        <NumberInput
          label={"Step acc. magnitude (reentry, run_type=1)"}
          value={runParams.step_acc_mag}
          onChange={v => setRunParams(p => ({ ...p, step_acc_mag: v }))}
        />
        <NumberInput
          label={"Step acc. height (m, reentry, run_type=1)"}
          value={runParams.step_acc_hgt}
          onChange={v => setRunParams(p => ({ ...p, step_acc_hgt: v }))}
        />
        <NumberInput
          label={"Step acc. duration (s, reentry, run_type=1)"}
          value={runParams.step_acc_dur}
          onChange={v => setRunParams(p => ({ ...p, step_acc_dur: v }))}
        />
      </div>
    </div>
  );
}
