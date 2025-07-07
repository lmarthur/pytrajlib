import { useState, useEffect } from "react";

const LocationBar = (params) => {
  const [value, setValue] = useState(params.value || "");
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");

  useEffect(() => {
    setValue(params.value || "");
  }, [params.value]);

  // Helper to check if value is valid lat,lon
  const isValidLatLon = (val) => {
    const parts = val.split(",");
    if (parts.length === 2) {
      const lat = parts[0].trim();
      const lon = parts[1].trim();
      return lat !== "" && lon !== "" && !isNaN(Number(lat)) && !isNaN(Number(lon));
    }
    return false;
  };

  // Geocode address using Nominatim
  const geocodeAddress = async (input) => {
    setLoading(true);
    setError("");
    try {
      const url = `https://nominatim.openstreetmap.org/search?q=${encodeURIComponent(input)}&format=geocodejson`;
      const res = await fetch(url);
      if (!res.ok) throw new Error("Geocoding failed");
      const data = await res.json();
      if (
        data.features &&
        data.features.length > 0 &&
        data.features[0].geometry &&
        data.features[0].geometry.coordinates
      ) {
        const [lon, lat] = data.features[0].geometry.coordinates;
        setValue(`${lat},${lon}`);
        params.onChange({ lat: Number(lat), lon: Number(lon) });
      } else {
        setError("No results found");
        params.onChange({ lat: null, lon: null });
      }
    } catch (e) {
      setError("Geocoding error");
      params.onChange({ lat: null, lon: null });
    }
    setLoading(false);
  };

  // Handler for blur or enter
  const handleInput = async () => {
    if (isValidLatLon(value)) {
      const [lat, lon] = value.split(",").map(s => Number(s.trim()));
      params.onChange({ lat, lon });
    } else if (value.trim() !== "") {
      await geocodeAddress(value);
    } else {
      params.onChange({ lat: null, lon: null });
    }
  };

  return (
    <div className="mb-6 text-sm">
      <div className="flex gap-2">
        <input
          type="text"
          placeholder={`Select or Search ${params.name}`}
          className="flex-1 px-4 py-2 rounded-full bg-white text-gray-900 focus:outline-none duration-400 border border-gray-200 focus:ring-2 focus:ring-cyan-400 text-sm"
          value={value}
          onChange={e => setValue(e.target.value)}
          onBlur={handleInput}
          onKeyDown={async e => {
            if (e.key === "Enter") {
              await handleInput();
            }
          }}
          disabled={loading}
        />
      </div>
      {error && <div className="text-xs text-red-500 mt-1">{error}</div>}
    </div>
  );
};
export default LocationBar;
