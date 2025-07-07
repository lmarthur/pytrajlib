"use client";
import dynamic from "next/dynamic";
import { useMemo, createContext, useContext, useState, useEffect } from "react";
import Sidebar from "@/app/components/sidebarComponents/Sidebar";

const MapContext = createContext();

export function MapProvider({ children }) {
  const [clickLoc, setClickLoc] = useState({ lat: null, lon: null });
  const [aimpoint, setAimpoint] = useState({ lat: null, lon: null });
  const [launchpoint, setLaunchpoint] = useState({ lat: null, lon: null });
  const [strikepoints, setStrikepoints] = useState([]);
  const [trajectoryData, setTrajectoryData] = useState([]);
  const [cep, setCEP] = useState(null);
  // Loading progress state for overlay
  const [loadingProgress, setLoadingProgress] = useState(0);

  useEffect(() => {
    window.setLoadingProgress = setLoadingProgress;
    return () => { delete window.setLoadingProgress; };
  }, []);

  useEffect(() => {
    console.log("loading progress", loadingProgress);
  }, [loadingProgress]);
  console.log("loading progress", loadingProgress);
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
        trajectoryData,
        setTrajectoryData,
        cep,
        setCEP,
        loadingProgress,
        setLoadingProgress,
      }}
    >
      {/* Loading overlay */}
      {(loadingProgress > 0 && loadingProgress < 1) && (
        <div className="fixed inset-0 flex items-center justify-center z-9999">
          <div
            className="w-1/4 h-5 border-solid border-4 bg-gray-200 rounded border-black"
          >
            <div
              className="h-3 bg-cyan-500 transition-all"
              style={{ width: `${loadingProgress * 100}%` }}
            />
          </div>
        </div>
      )}
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
