mod bindings;
mod game_controller_return_message;
mod game_controller_state_message;
mod visual_referee_message;

use std::{
    fmt::{self, Display, Formatter},
    str::FromStr,
    time::Duration,
};

use color_eyre::{eyre::WrapErr, Report, Result};
use coordinate_systems::Field;
use linear_algebra::{Point2, Pose2};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

pub use game_controller_return_message::GameControllerReturnMessage;
pub use game_controller_state_message::{
    GameControllerStateMessage, GamePhase, GameState, Half, Penalty, PenaltyShoot, Player,
    SubState, Team, TeamColor, TeamState,
};
pub use visual_referee_message::{VisualRefereeDecision, VisualRefereeMessage};

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
pub struct HulkMessage {
    pub jersey_number: JerseyNumber,
    pub pose: Pose2<Field>,
    pub is_referee_ready_signal_detected: bool,
    pub ball_position: Option<BallPosition<Field>>,
    pub time_to_reach_kick_position: Option<Duration>,
}

#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    Deserialize,
    PathDeserialize,
    PathIntrospect,
    PathSerialize,
    Serialize,
)]
pub struct BallPosition<Frame> {
    pub position: Point2<Frame>,
    pub age: Duration,
}

pub const HULKS_TEAM_NUMBER: u8 = 24;

#[derive(
    Clone,
    Copy,
    Debug,
    Deserialize,
    Eq,
    Hash,
    PartialEq,
    PathDeserialize,
    PathIntrospect,
    PathSerialize,
    Serialize,
)]
pub struct JerseyNumber {
    pub number: u8,
}
impl Default for JerseyNumber {
    fn default() -> Self {
        JerseyNumber { number: 7 }
    }
}

impl FromStr for JerseyNumber {
    type Err = Report;

    fn from_str(input: &str) -> Result<Self> {
        Ok(Self {
            number: input.parse().wrap_err("failed to parse JerseyNumber")?,
        })
    }
}

impl Display for JerseyNumber {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        self.number.fmt(formatter)
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use linear_algebra::{Point, Pose2};

    use crate::{BallPosition, HulkMessage, JerseyNumber};

    #[test]
    fn maximum_hulk_message_size() {
        let test_message = HulkMessage {
            jersey_number: JerseyNumber { number: 18 },
            pose: Pose2::default(),
            is_referee_ready_signal_detected: false,
            ball_position: Some(BallPosition {
                position: Point::origin(),
                age: Duration::MAX,
            }),
            time_to_reach_kick_position: Some(Duration::MAX),
        };
        assert!(bincode::serialize(&test_message).unwrap().len() <= 128)
    }
}
