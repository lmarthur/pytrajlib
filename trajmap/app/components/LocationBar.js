import { useState, useEffect } from "react";

const LocationBar = (params) => {
  const [value, setValue] = useState(params.value || "");

  useEffect(() => {
    setValue(params.value || "");
  }, [params.value]);

  return (
    <div className="mb-6">
      <input
        type="text"
        placeholder={`Select ${params.name}`}
        className="px-4 py-2 rounded-full bg-white text-gray-900 focus:outline-none  duration-400 border border-gray-200 focus:ring-2 focus:ring-cyan-400"
        value={value}
        onChange={(e) => {
          setValue(e.target.value);
          const lat = parseFloat(e.target.value.split(",")[0]);
          const lon = parseFloat(e.target.value.split(",")[1]);
          if (!isNaN(lat) && !isNaN(lon)) {
            params.onChange({ lat: lat, lon: lon });
          } else {
            params.onChange({ lat: null, lon: null });
          }
        }}
      />
    </div>
  );
};
export default LocationBar;
