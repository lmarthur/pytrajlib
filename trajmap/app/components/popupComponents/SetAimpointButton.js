import { useMapContext } from "@/app/page";
import PopupButton from "@/app/components/popupComponents/PopupButton";

const SetAimpointButton = (params) => {
  const { clickLoc, setAimpoint } = useMapContext();
  const handleClick = () => {
    setAimpoint({
      lat: clickLoc.lat,
      lon: clickLoc.lon,
    });
  };

  return (
    <div className="flex flex-col items-center justify-center">
      <PopupButton onClick={handleClick} name="Set Aimpoint" {...params} />
    </div>
  );
};

export default SetAimpointButton;
