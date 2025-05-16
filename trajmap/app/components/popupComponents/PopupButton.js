import Button from "@/app/components/Button";
const PopupButton = (params) => {
  let bg;
  if (params.active) {
    bg = "bg-cyan-500";
  } else {
    bg = "bg-gray-500";
  }

  return (
    <div className="flex flex-col items-center justify-center">
      <Button
        name={params.name}
        bg={bg}
        onClick={() => {
          params.onClick();
          params.setPopupVisible(false);
        }}
      />
    </div>
  );
};
export default PopupButton;
