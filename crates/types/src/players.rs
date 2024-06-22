use std::ops::{Index, IndexMut};

use color_eyre::Result;
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};
use spl_network_messages::{JerseyNumber, Penalty, TeamState};

#[derive(
    Clone,
    Copy,
    Default,
    Debug,
    Deserialize,
    Serialize,
    PathSerialize,
    PathIntrospect,
    PathDeserialize,
)]
pub struct Players<T> {
    pub one: T,
    pub two: T,
    pub three: T,
    pub four: T,
    pub five: T,
    pub six: T,
    pub seven: T,
}

impl<T> Index<JerseyNumber> for Players<T> {
    type Output = T;

    fn index(&self, index: JerseyNumber) -> &Self::Output {
        match index {
            JerseyNumber::One => &self.one,
            JerseyNumber::Two => &self.two,
            JerseyNumber::Three => &self.three,
            JerseyNumber::Four => &self.four,
            JerseyNumber::Five => &self.five,
            JerseyNumber::Six => &self.six,
            JerseyNumber::Seven => &self.seven,
        }
    }
}

impl<T> IndexMut<JerseyNumber> for Players<T> {
    fn index_mut(&mut self, index: JerseyNumber) -> &mut Self::Output {
        match index {
            JerseyNumber::One => &mut self.one,
            JerseyNumber::Two => &mut self.two,
            JerseyNumber::Three => &mut self.three,
            JerseyNumber::Four => &mut self.four,
            JerseyNumber::Five => &mut self.five,
            JerseyNumber::Six => &mut self.six,
            JerseyNumber::Seven => &mut self.seven,
        }
    }
}

impl From<TeamState> for Players<Option<Penalty>> {
    fn from(team_state: TeamState) -> Self {
        Self {
            one: team_state.players[0].penalty,
            two: team_state.players[1].penalty,
            three: team_state.players[2].penalty,
            four: team_state.players[3].penalty,
            five: team_state.players[4].penalty,
            six: team_state
                .players
                .get(5)
                .map(|player| player.penalty)
                .unwrap_or_default(),
            seven: team_state
                .players
                .get(6)
                .map(|player| player.penalty)
                .unwrap_or_default(),
        }
    }
}

#[derive(Clone, Copy)]
pub struct PlayersIterator<'a, T> {
    data: &'a Players<T>,
    next_forward: Option<JerseyNumber>,
    next_back: Option<JerseyNumber>,
}

impl<'a, T> PlayersIterator<'a, T> {
    fn new(data: &'a Players<T>) -> Self {
        Self {
            data,
            next_forward: Some(JerseyNumber::One),
            next_back: Some(JerseyNumber::Seven),
        }
    }
}

impl<'a, T> Iterator for PlayersIterator<'a, T> {
    type Item = (JerseyNumber, &'a T);
    fn next(&mut self) -> Option<Self::Item> {
        let result = self.next_forward.map(|number| (number, &self.data[number]));
        if self.next_forward == self.next_back {
            self.next_forward = None;
            self.next_back = None;
        }
        self.next_forward = match self.next_forward {
            Some(JerseyNumber::One) => Some(JerseyNumber::Two),
            Some(JerseyNumber::Two) => Some(JerseyNumber::Three),
            Some(JerseyNumber::Three) => Some(JerseyNumber::Four),
            Some(JerseyNumber::Four) => Some(JerseyNumber::Five),
            Some(JerseyNumber::Five) => Some(JerseyNumber::Six),
            Some(JerseyNumber::Six) => Some(JerseyNumber::Seven),
            Some(JerseyNumber::Seven) => None,
            None => None,
        };
        result
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let consumed_forward = match self.next_forward {
            Some(JerseyNumber::One) => 0,
            Some(JerseyNumber::Two) => 1,
            Some(JerseyNumber::Three) => 2,
            Some(JerseyNumber::Four) => 3,
            Some(JerseyNumber::Five) => 4,
            Some(JerseyNumber::Six) => 5,
            Some(JerseyNumber::Seven) => 6,
            None => 7,
        };
        let consumed_back = match self.next_back {
            Some(JerseyNumber::One) => 6,
            Some(JerseyNumber::Two) => 5,
            Some(JerseyNumber::Three) => 4,
            Some(JerseyNumber::Four) => 3,
            Some(JerseyNumber::Five) => 2,
            Some(JerseyNumber::Six) => 1,
            Some(JerseyNumber::Seven) => 0,
            None => 7,
        };
        let remaining = 7usize.saturating_sub(consumed_forward + consumed_back);
        (remaining, Some(remaining))
    }
}

impl<'a, T> DoubleEndedIterator for PlayersIterator<'a, T> {
    fn next_back(&mut self) -> Option<Self::Item> {
        let result = self.next_back.map(|number| (number, &self.data[number]));
        if self.next_forward == self.next_back {
            self.next_forward = None;
            self.next_back = None;
        }
        self.next_back = match self.next_back {
            Some(JerseyNumber::One) => None,
            Some(JerseyNumber::Two) => Some(JerseyNumber::One),
            Some(JerseyNumber::Three) => Some(JerseyNumber::Two),
            Some(JerseyNumber::Four) => Some(JerseyNumber::Three),
            Some(JerseyNumber::Five) => Some(JerseyNumber::Four),
            Some(JerseyNumber::Six) => Some(JerseyNumber::Five),
            Some(JerseyNumber::Seven) => Some(JerseyNumber::Six),
            None => None,
        };
        result
    }
}

impl<'a, T> ExactSizeIterator for PlayersIterator<'a, T> {
    // The default implementation only requires `Iterator::size_hint()` to be exact
}

impl<T> Players<T> {
    pub fn iter(&self) -> PlayersIterator<'_, T> {
        PlayersIterator::new(self)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn exact_size() {
        let players = Players::<i32>::default();
        let mut iterator = players.iter();

        assert_eq!(iterator.len(), 7);
        iterator.next();
        assert_eq!(iterator.len(), 6);
        iterator.next();
        assert_eq!(iterator.len(), 5);
        iterator.next();
        assert_eq!(iterator.len(), 4);
        iterator.next();
        assert_eq!(iterator.len(), 3);
        iterator.next();
        assert_eq!(iterator.len(), 2);
        iterator.next();
        assert_eq!(iterator.len(), 1);
        iterator.next();
        assert_eq!(iterator.len(), 0);
        iterator.next();
        assert_eq!(iterator.len(), 0);
        iterator.next();
    }

    #[test]
    fn double_ended() {
        let players = Players {
            one: 1,
            two: 2,
            three: 3,
            four: 4,
            five: 5,
            six: 6,
            seven: 7,
        };
        let mut iterator = players.iter();

        assert_eq!(iterator.len(), 7);
        assert_eq!(iterator.next(), Some((JerseyNumber::One, &1)));
        assert_eq!(iterator.len(), 6);
        assert_eq!(iterator.next(), Some((JerseyNumber::Two, &2)));
        assert_eq!(iterator.len(), 5);
        assert_eq!(iterator.next_back(), Some((JerseyNumber::Seven, &7)));
        assert_eq!(iterator.len(), 4);
        assert_eq!(iterator.next_back(), Some((JerseyNumber::Six, &6)));
        assert_eq!(iterator.len(), 3);
        assert_eq!(iterator.next(), Some((JerseyNumber::Three, &3)));
        assert_eq!(iterator.len(), 2);
        assert_eq!(iterator.next(), Some((JerseyNumber::Four, &4)));
        assert_eq!(iterator.len(), 1);
        assert_eq!(iterator.next_back(), Some((JerseyNumber::Five, &5)));
        assert_eq!(iterator.len(), 0);
        assert_eq!(iterator.next(), None);
        assert_eq!(iterator.len(), 0);
        assert_eq!(iterator.next_back(), None);
        assert_eq!(iterator.len(), 0);
        assert_eq!(iterator.next(), None);
        assert_eq!(iterator.len(), 0);
        assert_eq!(iterator.next_back(), None);
        assert_eq!(iterator.len(), 0);
    }
}
