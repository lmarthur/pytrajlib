"use client";
import { MapContainer, Marker, TileLayer, Tooltip } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import { useEffect, useState } from "react";
import { useMap, useMapEvent, Polyline, ScaleControl, Circle } from "react-leaflet";
import LocationPopup from "./popupComponents/LocationPopup";
import { useMapContext } from "../page";
import StrikeMarker from "./StrikeMarker";
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

const UpdatePositionAndZoom = (props) => {
    const map = useMap();
  useEffect(() => {
    if (!props || !props.aimpoint.lat || !props.aimpoint.lon) {
      return;
    }
    console.log("Updating map view to aimpoint 2:", props);
    let zoom = 2;
    if (props.cep > 10000) {
      zoom = 8;
    }
    else if (props.cep > 1000) {
      zoom = 12;
    }
    else if (props.cep > 100) {
      zoom = 15;
    }
    else {
      zoom = 17;
    }
    props.setShowMarkers(false);
    const duration = 2; // seconds
    map.flyTo([props.aimpoint.lat, props.aimpoint.lon], zoom, {duration: duration});
    setTimeout(() => 
      {
        props.setShowMarkers(true);
      }, duration * 1000 - 500);
  }, [props.cep]);
}

export default function Map(props) {
  const { position, zoom } = props;
  const {
    clickLoc,
    setClickLoc,
    launchpoint,
    aimpoint,
    strikepoints,
    trajectoryData,
    cep,
  } = useMapContext();
  const [popupVisible, setPopupVisible] = useState(false);
  const [StrikepointMarkers, setStrikepointMarkers] = useState([]);
  const [trajectoryLine, setTrajectoryLine] = useState(null);
  const [cepLine, setCepLine] = useState(null);
  const [showMarkers, setShowMarkers] = useState(true);


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
    Update the strikepoint markers when the strikepoints change.
    */
    const newMarkers = [];
    for (let i = 0; i < strikepoints.length; i++) {
      newMarkers.push(
        <StrikeMarker key={i} position={strikepoints[i]}>
          <Tooltip permanent>Strike Point {i + 1}</Tooltip>
        </StrikeMarker>
      );
    }
    setStrikepointMarkers(newMarkers);
  }, [strikepoints]);

  useEffect(() => {
    /*
    Update the trajectory line when the trajectory data changes.
    */
    if (trajectoryData && trajectoryData.length > 0) {
      console.log("Trajectory data:", trajectoryData);
      const trajectoryLine = trajectoryData.map(([t, lat, lon]) => [lat, lon]);
      setTrajectoryLine(
        <>
          <Polyline positions={trajectoryLine} color="red" weight={3} />
        </>
      );
    } else {
      setTrajectoryLine(null);
    }
  }
    , [trajectoryData]);

  useEffect(() => {
    /*
    Draw a circle representing the CEP around the aimpoint.
    */
   if (!cep) {
    return;
   }
      setCepLine(
        <Circle
          center={[aimpoint.lat, aimpoint.lon]}
          radius={cep}
          pathOptions={{
            color: "black",
            dashArray: "5, 5",
            fill: true,
            fillColor: "black",
            fillOpacity: 0.1,
            weight: 1,
          }}
        >
          <Tooltip permanent>{`CEP: ${cep.toFixed(2)} m`}</Tooltip>
        </Circle>
      );
  }, [cep]);

  const AimpointMarker =
    aimpoint.lat !== null && aimpoint.lon !== null ? (
      <Marker position={[aimpoint.lat, aimpoint.lon]}>
        <Tooltip>Aim Point</Tooltip>
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
        <UpdatePositionAndZoom aimpoint={aimpoint} cep={cep} setShowMarkers={setShowMarkers}/>
        <MapClickHandler />
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        {showMarkers ? LaunchpointMarker: null}
        {showMarkers ? trajectoryLine: null}
        {showMarkers ? StrikepointMarkers: null}
        {showMarkers ? AimpointMarker: null}
        {showMarkers ? cepLine: null}
        <ScaleControl position="bottomright" />
      </MapContainer>
      {popupVisible ? (
        <LocationPopup setPopupVisible={setPopupVisible} />
      ) : null}
    </div>
  );
}
