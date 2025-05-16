import { useMapContext } from "@/app/page";
import PopupButton from "@/app/components/popupComponents/PopupButton";

const SetLaunchpointButton = (params) => {
  const { clickLoc, setLaunchpoint } = useMapContext();
  const handleClick = () => {
    setLaunchpoint({
      lat: clickLoc.lat,
      lon: clickLoc.lon,
    });
  };

  return (
    <div className="flex flex-col items-center justify-center">
      <PopupButton onClick={handleClick} name="Set Launch Point" {...params} />
    </div>
  );
};

export default SetLaunchpointButton;
