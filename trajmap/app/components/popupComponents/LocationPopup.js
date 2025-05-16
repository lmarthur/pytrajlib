import { useMapContext } from "@/app/page";
import SetAimpointButton from "./SetAimpointButton";
import SetLaunchpointButton from "./SetLaunchpointButton";

const LocationPopup = (params) => {
  const { clickLoc, aimpoint, launchpoint } = useMapContext();
  return (
    <div className="fixed left-1/2 bottom-8 bg-white text-gray-800 rounded-lg shadow-lg px-6 py-4 border border-gray-200 z-1000 flex flex-col items-center">
      <div className="mb-2 text-center">
        {clickLoc.lat.toFixed(5)}, {clickLoc.lon.toFixed(5)}
      </div>
      <div className="flex flex-row justify-center gap-4">
        <SetLaunchpointButton active={launchpoint.lat == null} {...params} />
        <SetAimpointButton
          active={aimpoint.lat == null && launchpoint.lat != null}
          {...params}
        />
      </div>
      <button
        className="absolute top-0 right-2 text-gray-400 hover:text-gray-700 text-xl cursor-pointer"
        onClick={() => params.setPopupVisible(false)}
        aria-label="Close"
        type="button"
      >
        ×
      </button>
    </div>
  );
};

export default LocationPopup;
