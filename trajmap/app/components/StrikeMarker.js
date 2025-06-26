import { Marker } from "react-leaflet";
import L from "leaflet";

const xIcon = L.divIcon({
  className: "",
  html: `
    <svg width="24" height="24" viewBox="0 0 32 32" xmlns="http://www.w3.org/2000/svg">
        <line x1="8" y1="8" x2="24" y2="24" stroke="black" stroke-width="3" stroke-linecap="round"/>
        <line x1="24" y1="8" x2="8" y2="24" stroke="black" stroke-width="3" stroke-linecap="round"/>
    </svg>


      `,
  iconSize: [32, 32],
  iconAnchor: [16, 16],
});

const StrikeMarker = (params) => {
  return <Marker position={params.position} icon={xIcon} />;
};

export default StrikeMarker;
