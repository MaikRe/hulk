use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};
use serialize_hierarchy::SerializeHierarchy;

use linear_algebra::{Point2, Vector2};

#[derive(Clone, Copy, Serialize, Deserialize, SerializeHierarchy, Debug)]
pub struct BallPosition<Frame> {
    pub position: Point2<Frame>,
    pub velocity: Vector2<Frame>,
    pub last_seen: SystemTime,
}

impl<Frame> Default for BallPosition<Frame> {
    fn default() -> Self {
        Self {
            position: Default::default(),
            velocity: Default::default(),
            last_seen: UNIX_EPOCH,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, SerializeHierarchy)]
pub struct HypotheticalBallPosition<Frame> {
    pub position: Point2<Frame>,
    pub validity: f32,
}
