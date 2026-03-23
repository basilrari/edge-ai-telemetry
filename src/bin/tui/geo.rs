//! Waypoint input parsing and geodesic helpers.

/// Parse "lat lon alt" (three numbers) or "alt" (one number; uses current lat/lon).
pub(crate) fn parse_waypoint_input(
    s: &str,
    current_lat: Option<f64>,
    current_lon: Option<f64>,
    _current_alt: Option<f64>,
) -> Result<(f64, f64, f64), String> {
    let parts: Vec<&str> = s.split_whitespace().collect();
    if parts.is_empty() {
        return Err("empty".to_string());
    }
    if parts.len() == 1 {
        let alt: f64 = parts[0].parse().map_err(|_| "alt must be a number")?;
        let lat = current_lat.ok_or("current position (GPS) needed for 'alt only'")?;
        let lon = current_lon.ok_or("current position (GPS) needed for 'alt only'")?;
        return Ok((lat, lon, alt));
    }
    if parts.len() != 3 {
        return Err("use: lat lon alt (space-sep), or just alt".to_string());
    }
    let lat: f64 = parts[0].parse().map_err(|_| "lat must be a number")?;
    let lon: f64 = parts[1].parse().map_err(|_| "lon must be a number")?;
    let alt: f64 = parts[2].parse().map_err(|_| "alt must be a number")?;
    if !(-90.0..=90.0).contains(&lat) {
        return Err("lat must be -90..90".to_string());
    }
    if !(-180.0..=180.0).contains(&lon) {
        return Err("lon must be -180..180".to_string());
    }
    Ok((lat, lon, alt))
}

pub(crate) fn horizontal_distance_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let lat1_rad = lat1.to_radians();
    let lat2_rad = lat2.to_radians();
    let a = dlat.sin().mul_add(
        dlat.sin(),
        dlon.sin() * dlon.sin() * lat1_rad.cos() * lat2_rad.cos(),
    );
    let a = a.min(1.0).max(0.0);
    let c = 2.0 * (1.0 - a).sqrt().atan2(a.sqrt());
    6371000.0 * c // Earth radius in meters
}
