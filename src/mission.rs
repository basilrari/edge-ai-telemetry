//! Mission types: waypoints and commands (protocol-agnostic).

use serde::{Deserialize, Serialize};

/// A waypoint command (protocol-agnostic).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum WaypointCommand {
    /// Go to waypoint.
    GoTo,
    /// Takeoff.
    Takeoff,
    /// Land.
    Land,
    /// Other / custom (e.g. for extensibility).
    Other(u8),
}

/// A single waypoint with position and metadata.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Waypoint {
    /// Latitude in degrees.
    pub latitude: f64,
    /// Longitude in degrees.
    pub longitude: f64,
    /// Altitude (e.g. meters AMSL or relative).
    pub altitude: f32,
    /// Sequence index in the mission.
    pub sequence: u16,
}

impl Waypoint {
    /// Latitude as integer (degrees × 10^7).
    pub fn lat_int(&self) -> i32 {
        (self.latitude * 1e7).round() as i32
    }

    /// Longitude as integer (degrees × 10^7).
    pub fn lon_int(&self) -> i32 {
        (self.longitude * 1e7).round() as i32
    }
}

/// A mission: ordered collection of waypoints.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Mission {
    /// Waypoints in order.
    pub waypoints: Vec<Waypoint>,
}

impl Mission {
    /// Total number of items (waypoints) in the mission.
    pub fn total_items(&self) -> usize {
        self.waypoints.len()
    }
}
