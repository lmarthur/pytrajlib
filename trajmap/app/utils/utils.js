/**
 * Converts spherical coordinates to Cartesian coordinates.
 * @param {number[]} spherCoords - Array of spherical coordinates [r, long, lat] (rangles in adians).
 * @returns {number[]} Cartesian coordinates [x, y, z] (meters).
 */
export const sphere2cart = (spherCoords) => {
  const [r, lon, lat] = spherCoords;
  const x = r * Math.cos(lon) * Math.cos(lat);
  const y = r * Math.sin(lon) * Math.cos(lat);
  const z = r * Math.sin(lat);
  return [x, y, z];
}

/**
 * Converts Cartesian coordinates to spherical coordinates.
 * @param {number} x - Cartesian x-coordinate (meters).
 * @param {number} y - Cartesian y-coordinate (meters).
 * @param {number} z - Cartesian z-coordinate (meters).
 * @returns {number[]} Spherical coordinates [r, longitude, latitude] (r in meters, angles in radians).
 */
export const cart2sphere = (x, y, z) => {
  // cartCoords: [x, y, z]
  // returns [r, longitude, latitude]

  const r = Math.sqrt(x * x + y * y + z * z);
  const longitude = Math.atan2(y, x); // in radians
  const latitude = Math.atan(z / Math.sqrt(x * x + y * y)); // in radians

  return [r, longitude, latitude];
};

/**
 * Calculate the bearing (in radians) from start to end (lat, lon in radians)
 */
const calc_bearing = (start, end) => {

  const [launch_lat, launch_lon] = start;
  const [aim_lat, aim_lon] = end;
  const lon_diff = aim_lon - launch_lon;

  // East component
  const east = Math.sin(lon_diff) * Math.cos(aim_lat);

  // North component
  const north =
    Math.cos(launch_lat) * Math.sin(aim_lat) -
    Math.sin(launch_lat) * Math.cos(aim_lat) * Math.cos(lon_diff);


  return Math.atan2(north, east);
}

/**
 * Calculate the haversine distance between two points (lat, lon in radians)
 */
const haversineDistance = (start, end) => {
  const [launch_lat, launch_lon] = start;
  const [aim_lat, aim_lon] = end;
  const a =
    Math.sin((aim_lat - launch_lat) / 2) * Math.sin((aim_lat - launch_lat) / 2) +
    Math.cos(launch_lat) * Math.cos(aim_lat) *
    Math.sin((aim_lon - launch_lon) / 2) * Math.sin((aim_lon - launch_lon) / 2);
  const angular_distance = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  const distance = 6371e3 * angular_distance;
  return distance;
};

/**
 * Calculate the end location (lat, lon in radians) given a start location (lat, lon in radians),
 * a bearing (in radians), and a distance (in meters).
 */
const getLocation = (bearing, distance, start) => {
  // Formula assumes angle is clockwise from north
  bearing = -(bearing - Math.PI / 2);
  const launch_lat = start[0]; // latitude of start point in radians
  const launch_long = start[1]; // longitude of start point in radians
  const angular_distance = distance / 6371e3;
  const aim_lat = Math.asin(Math.sin(launch_lat) * Math.cos(angular_distance) +
    Math.cos(launch_lat) * Math.sin(angular_distance) * Math.cos(bearing));
  const aim_long = launch_long + Math.atan2(Math.sin(bearing) * Math.sin(angular_distance) * Math.cos(launch_lat),
    Math.cos(angular_distance) - Math.sin(launch_lat) * Math.sin(aim_lat));
  return [aim_lat, aim_long];
};

/**
 * Tranform from the cartesian x, y, z impact points via a trajectory from the 
 * launchpoint to the "Earth" coordinates launched from the user-selected launchpoint.
 */
export const transformToEarthCoords = (x, y, z, launchpoint) => {
  const launch_lon = (launchpoint.lon * Math.PI) / 180;
  const launch_lat = (launchpoint.lat * Math.PI) / 180;

  const [r, long_from_origin, lat_from_origin] = cart2sphere(x, y, z);
  // Find bearing from origin to strikepoint
  const bearing = calc_bearing([0, 0], [lat_from_origin, long_from_origin]);
  // Find distance from origin to strikepoint
  const distance = haversineDistance([0, 0], [lat_from_origin, long_from_origin]);
  // Get the Earth strikepoint location
  const latLon = getLocation(bearing, distance, [launch_lat, launch_lon]);
  return latLon;
};

const getLocalImpact = (lat, lon, aim_lat, aim_lon) => {
  const [x_aim, y_aim, z_aim] = sphere2cart([6371e3, aim_lon, aim_lat]);
  let [impact_x, impact_y, impact_z] = sphere2cart([6371e3, lon, lat]);
  impact_x = impact_x - x_aim;
  impact_y = impact_y - y_aim;
  impact_z = impact_z - z_aim;

  const impact_x_local = -Math.sin(aim_lon) * impact_x + Math.cos(aim_lat) * impact_y;
  const impact_y_local = (
    -Math.sin(aim_lat) * Math.cos(aim_lon) * impact_x
    - Math.sin(aim_lat) * Math.sin(aim_lat) * impact_y
    + Math.cos(aim_lat) * impact_z
  )

  return [impact_x_local, impact_y_local]
};

/**
 * Get the Circular Error Probable (CEP) from the impact data.
 */
export const calculateCEP = (impactData, launchpoint) => {
  const [x_aim, y_aim, z_aim] = impactData[0];
  const [r, aim_lon, aim_lat] = cart2sphere(x_aim, y_aim, z_aim);

  const miss_distances = [];
  for (let row of impactData.slice(1)) {
    const x = row[1];
    const y = row[2];
    const z = row[3];
    const [lat, lon] = transformToEarthCoords(x, y, z, launchpoint)
    const [impact_x_local, impact_y_local] = getLocalImpact(lat, lon, aim_lat, aim_lon);
    miss_distances.push(Math.sqrt(impact_x_local ** 2 + impact_y_local ** 2));
  }
  // Calculate the 50th percentile
  miss_distances.sort((a, b) => a - b);
  const mid = Math.floor(miss_distances.length / 2);
  const cep = miss_distances.length % 2 !== 0
    ? miss_distances[mid]
    : (miss_distances[mid - 1] + miss_distances[mid]) / 2;

  return cep;
};