"use client";
import { MapContainer, Marker, TileLayer, Tooltip } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import { useEffect, useState } from "react";
import { useMap, useMapEvent } from "react-leaflet";
import LocationPopup from "./popupComponents/LocationPopup";
import { useMapContext } from "../page";
import "leaflet-defaulticon-compatibility";
import "leaflet-defaulticon-compatibility/dist/leaflet-defaulticon-compatibility.css";

const RemoveAttributionPrefix = () => {
  /*
  Removes the flag from the lower right corner.
  */
  const map = useMap();
  useEffect(() => {
    if (map && map.attributionControl) {
      map.attributionControl.setPrefix("");
    }
  }, [map]);
  return null;
};

export default function Map(props) {
  const { position, zoom } = props;
  const {
    clickLoc,
    setClickLoc,
    launchpoint,
    aimpoint,
    simAimpoint,
    strikepoints,
  } = useMapContext();
  const [popupVisible, setPopupVisible] = useState(false);
  const [StrikepointMarkers, setStrikepointMarkers] = useState([]);
  const [SimAimpointMarker, setSimAimpointMarker] = useState("");

  const MapClickHandler = () => {
    useMapEvent("click", (e) => {
      setClickLoc({ lat: e.latlng.lat, lon: e.latlng.lng });
      console.log("Map clicked at:", e.latlng.lat, e.latlng.lng);
      setPopupVisible(true);
    });

    useEffect(() => {
      /*
    Close the popup when the escape key is pressed.
    */
      const handleKeyDown = (e) => {
        if (e.key === "Escape") {
          setPopupVisible(false);
        }
      };
      window.addEventListener("keydown", handleKeyDown);
      return () => window.removeEventListener("keydown", handleKeyDown);
    }, []);

    return null;
  };

  useEffect(() => {
    /*
    Update the aimpoint marker when the aimpoint changes.
    */
    if (simAimpoint.lat !== null && simAimpoint.lon !== null) {
      setSimAimpointMarker(
        <Marker position={[simAimpoint.lat, simAimpoint.lon]}>
          <Tooltip permanent>Simulation Aim Point</Tooltip>
        </Marker>
      );
    } else {
      setSimAimpointMarker(null);
    }
  }, [simAimpoint]);

  useEffect(() => {
    /*
    Update the strikepoint markers when the strikepoints change.
    */
    const newMarkers = [];
    for (let i = 0; i < strikepoints.length; i++) {
      newMarkers.push(
        <Marker key={i} position={strikepoints[i]}>
          <Tooltip permanent>Strike Point {i + 1}</Tooltip>
        </Marker>
      );
    }
    setStrikepointMarkers(newMarkers);
  }, [strikepoints]);

  const AimpointMarker =
    aimpoint.lat !== null && aimpoint.lon !== null ? (
      <Marker position={[aimpoint.lat, aimpoint.lon]}>
        <Tooltip permanent>Aim Point</Tooltip>
      </Marker>
    ) : null;
  const LaunchpointMarker =
    launchpoint.lat !== null && launchpoint.lon !== null ? (
      <Marker position={[launchpoint.lat, launchpoint.lon]}>
        <Tooltip permanent>Launch Point</Tooltip>
      </Marker>
    ) : null;

  return (
    <div className="h-screen w-full">
      <MapContainer
        center={position}
        zoom={zoom}
        scrollWheelZoom={true}
        minZoom={2}
        worldCopyJump={true}
        style={{ height: "100vh", width: "100%" }}
      >
        <RemoveAttributionPrefix />
        <MapClickHandler />
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        {LaunchpointMarker}
        {AimpointMarker}
        {SimAimpointMarker}
        {StrikepointMarkers}
      </MapContainer>
      {popupVisible ? (
        <LocationPopup setPopupVisible={setPopupVisible} />
      ) : null}
    </div>
  );
}
