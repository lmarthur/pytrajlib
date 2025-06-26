/**
 * Converts spherical coordinates to Cartesian coordinates.
 * @param {number[]} spherCoords - Array of spherical coordinates [r, long, lat] (rangles in adians).
 * @returns {number[]} Cartesian coordinates [x, y, z] (meters).
 */
export const sphercoordsToCartcoords = (spherCoords) =>{
    const [r, lon, lat] = spherCoords;
    const x = r * Math.cos(lon) * Math.cos(lat);
    const y = r * Math.sin(lon) * Math.cos(lat);
    const z = r * Math.sin(lat);
    return [x, y, z];
}