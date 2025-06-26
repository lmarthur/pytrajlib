import { useMapContext } from "../../page";
import LocationBar from "../LocationBar";

const AimPointBar = () => {
  const { aimpoint, setAimpoint } = useMapContext();

  const value =
    aimpoint.lat !== null && aimpoint.lon !== null
      ? `${aimpoint.lat.toFixed(5)}, ${aimpoint.lon.toFixed(5)}`
      : null;
  return <LocationBar name="Aim Point" value={value} onChange={setAimpoint} />;
};

export default AimPointBar;
