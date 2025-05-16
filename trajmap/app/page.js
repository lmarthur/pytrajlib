"use client";
import dynamic from "next/dynamic";
import { useMemo, createContext, useContext, useState } from "react";
import Sidebar from "@/app/components/sidebarComponents/Sidebar";

const MapContext = createContext();

export function MapProvider({ children }) {
  const [clickLoc, setClickLoc] = useState({ lat: null, lon: null });
  const [aimpoint, setAimpoint] = useState({ lat: null, lon: null });
  const [launchpoint, setLaunchpoint] = useState({ lat: null, lon: null });
  const [strikepoints, setStrikepoints] = useState([]);

  // The simulation aimpoint is the aimpoint used in the simulation. It may
  // differ from the user-selected aimpoint due to an imperfect optimization.
  const [simAimpoint, setSimAimpoint] = useState({ lat: null, lon: null });

  return (
    <MapContext.Provider
      value={{
        clickLoc,
        setClickLoc,
        aimpoint,
        setAimpoint,
        launchpoint,
        setLaunchpoint,
        strikepoints,
        setStrikepoints,
        simAimpoint,
        setSimAimpoint,
      }}
    >
      {children}
    </MapContext.Provider>
  );
}

export function useMapContext() {
  return useContext(MapContext);
}

export default function MyPage() {
  const Map = useMemo(
    () =>
      dynamic(() => import("@/app/components/Map.client"), {
        loading: () => <p>Loading...</p>,
        ssr: false,
      }),
    []
  );

  return (
    <MapProvider>
      <div className="flex flex-row h-screen">
        <Sidebar />
        <Map position={[0, 0]} zoom={2} />
      </div>
    </MapProvider>
  );
}
