use std::ops::{Index, IndexMut};

use serde::{Deserialize, Serialize};
use spl_network::{Penalty, PlayerNumber, TeamState};

use crate::framework;

#[derive(Clone, Copy, Default, Debug, Deserialize, Serialize)]
pub struct Players<T> {
    pub one: T,
    pub two: T,
    pub three: T,
    pub four: T,
    pub five: T,
}

impl<T> Index<PlayerNumber> for Players<T> {
    type Output = T;

    fn index(&self, index: PlayerNumber) -> &Self::Output {
        match index {
            PlayerNumber::One => &self.one,
            PlayerNumber::Two => &self.two,
            PlayerNumber::Three => &self.three,
            PlayerNumber::Four => &self.four,
            PlayerNumber::Five => &self.five,
        }
    }
}

impl<T> IndexMut<PlayerNumber> for Players<T> {
    fn index_mut(&mut self, index: PlayerNumber) -> &mut Self::Output {
        match index {
            PlayerNumber::One => &mut self.one,
            PlayerNumber::Two => &mut self.two,
            PlayerNumber::Three => &mut self.three,
            PlayerNumber::Four => &mut self.four,
            PlayerNumber::Five => &mut self.five,
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
        }
    }
}

impl<T> crate::framework::SerializeHierarchy for Players<T>
where
    T: Serialize + for<'de> Deserialize<'de> + framework::SerializeHierarchy,
{
    fn serialize_hierarchy(&self, field_path: &str) -> anyhow::Result<serde_json::Value> {
        use anyhow::Context;
        let split = field_path.split_once('.');
        match split {
            Some((field_name, suffix)) => match field_name {
                "one" => self
                    .one
                    .serialize_hierarchy(suffix)
                    .context("Failed to serialize field `one`"),
                "two" => self
                    .two
                    .serialize_hierarchy(suffix)
                    .context("Failed to serialize field `two`"),
                "three" => self
                    .three
                    .serialize_hierarchy(suffix)
                    .context("Failed to serialize field `three`"),
                "four" => self
                    .four
                    .serialize_hierarchy(suffix)
                    .context("Failed to serialize field `four`"),
                "five" => self
                    .five
                    .serialize_hierarchy(suffix)
                    .context("Failed to serialize field `five`"),
                _ => anyhow::bail!("No such field in type: `{}`", field_path),
            },
            None => match field_path {
                "one" => serde_json::to_value(&self.one).context("Failed to serialize field `one`"),
                "two" => serde_json::to_value(&self.two).context("Failed to serialize field `two`"),
                "three" => {
                    serde_json::to_value(&self.three).context("Failed to serialize field `three`")
                }
                "four" => {
                    serde_json::to_value(&self.four).context("Failed to serialize field `four`")
                }
                "five" => {
                    serde_json::to_value(&self.five).context("Failed to serialize field `five`")
                }
                _ => anyhow::bail!("No such field in type: `{}`", field_path),
            },
        }
    }

    fn deserialize_hierarchy(
        &mut self,
        field_path: &str,
        data: serde_json::Value,
    ) -> anyhow::Result<()> {
        use anyhow::Context;
        let split = field_path.split_once('.');
        match split {
            Some((field_name, suffix)) => match field_name {
                "one" => self
                    .one
                    .deserialize_hierarchy(suffix, data)
                    .context("Failed to deserialize field `one`"),
                "two" => self
                    .two
                    .deserialize_hierarchy(suffix, data)
                    .context("Failed to deserialize field `two`"),
                "three" => self
                    .three
                    .deserialize_hierarchy(suffix, data)
                    .context("Failed to deserialize field `three`"),
                "four" => self
                    .four
                    .deserialize_hierarchy(suffix, data)
                    .context("Failed to deserialize field `four`"),
                "five" => self
                    .five
                    .deserialize_hierarchy(suffix, data)
                    .context("Failed to deserialize field `five`"),
                _ => anyhow::bail!("No such field in type: `{}`", field_path),
            },
            None => match field_path {
                "one" => {
                    self.one = serde_json::from_value(data)
                        .context("Failed to deserialize field `one`")?;
                    Ok(())
                }
                "two" => {
                    self.two = serde_json::from_value(data)
                        .context("Failed to deserialize field `two`")?;
                    Ok(())
                }
                "three" => {
                    self.three = serde_json::from_value(data)
                        .context("Failed to deserialize field `three`")?;
                    Ok(())
                }
                "four" => {
                    self.four = serde_json::from_value(data)
                        .context("Failed to deserialize field `four`")?;
                    Ok(())
                }
                "five" => {
                    self.five = serde_json::from_value(data)
                        .context("Failed to deserialize field `five`")?;
                    Ok(())
                }
                _ => anyhow::bail!("No such field in type: `{}`", field_path),
            },
        }
    }

    fn exists(field_path: &str) -> bool {
        let split = field_path.split_once('.');
        match split {
            Some((field_name, suffix)) => match field_name {
                "one" => T::exists(suffix),
                "two" => T::exists(suffix),
                "three" => T::exists(suffix),
                "four" => T::exists(suffix),
                "five" => T::exists(suffix),
                _ => false,
            },
            None => matches!(field_path, "one" | "two" | "three" | "four" | "five"),
        }
    }

    fn get_hierarchy() -> crate::framework::HierarchyType {
        let mut fields = std::collections::BTreeMap::new();
        fields.insert("one".to_string(), T::get_hierarchy());
        fields.insert("two".to_string(), T::get_hierarchy());
        fields.insert("three".to_string(), T::get_hierarchy());
        fields.insert("four".to_string(), T::get_hierarchy());
        fields.insert("five".to_string(), T::get_hierarchy());
        crate::framework::HierarchyType::Struct { fields }
    }
}
