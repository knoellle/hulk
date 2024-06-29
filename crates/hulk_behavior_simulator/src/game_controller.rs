use std::time::SystemTime;

use bevy::prelude::*;
use spl_network_messages::{GamePhase, GameState, Team};
use types::{game_controller_state::GameControllerState, players::Players};

#[derive(Resource)]
pub struct GameController {
    pub state: GameControllerState,
}

impl Default for GameController {
    fn default() -> Self {
        Self {
            state: GameControllerState {
                game_state: GameState::Initial,
                game_phase: GamePhase::Normal,
                kicking_team: Team::Hulks,
                last_game_state_change: SystemTime::UNIX_EPOCH,
                penalties: Players {
                    one: None,
                    two: None,
                    three: None,
                    four: None,
                    five: None,
                    six: None,
                    seven: None,
                },
                remaining_amount_of_messages: 1200,
                sub_state: None,
                hulks_team_is_home_after_coin_toss: true,
            },
        }
    }
}
