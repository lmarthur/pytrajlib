import { useMapContext } from "../../page";
import LocationBar from "../LocationBar";

const LaunchpointBar = () => {
  const { launchpoint, setLaunchpoint } = useMapContext();
  const value =
    launchpoint.lat !== null && launchpoint.lon !== null
      ? `${launchpoint.lat.toFixed(5)}, ${launchpoint.lon.toFixed(5)}`
      : null;
  return (
    <LocationBar name="Launch Point" value={value} onChange={setLaunchpoint} />
  );
};

export default LaunchpointBar;
