use std::{collections::HashMap, time::SystemTime};

use path_serde::{PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};
use spl_network_messages::Penalty;

#[derive(Debug, Clone, Serialize, Deserialize, PathSerialize, PathIntrospect, PartialEq)]

pub struct LastFilteredGameControllerStateChanges {
    pub game_state: SystemTime,
    pub opponent_game_state: SystemTime,
    pub game_phase: SystemTime,
    pub kicking_team: SystemTime,
    pub penalties: HashMap<usize, Option<SystemTime>>,
    pub opponent_penalties: HashMap<usize, Option<Penalty>>,
    pub sub_state: Option<SystemTime>,
}

impl Default for LastFilteredGameControllerStateChanges {
    fn default() -> Self {
        Self {
            game_state: SystemTime::now(),
            opponent_game_state: SystemTime::now(),
            game_phase: SystemTime::now(),
            kicking_team: SystemTime::now(),
            penalties: HashMap::new(),
            opponent_penalties: HashMap::new(),
            sub_state: None,
        }
    }
}
