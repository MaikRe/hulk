use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    Deserialize,
    Eq,
    PartialEq,
    Serialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
)]
pub enum Side {
    #[default]
    Left,
    Right,
}

impl Side {
    pub fn opposite(&self) -> Side {
        match self {
            Side::Left => Self::Right,
            Side::Right => Self::Left,
        }
    }
}

#[derive(
    Clone, Debug, Default, Deserialize, Serialize, PathSerialize, PathDeserialize, PathIntrospect,
)]
pub struct SupportFoot {
    pub support_side: Option<Side>,
    pub changed_this_cycle: bool,
}
