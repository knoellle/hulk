use std::{collections::HashSet, hash::BuildHasherDefault};

use nalgebra::Point2;
use serde::{Deserialize, Serialize};
use serialize_hierarchy::SerializeHierarchy;

use crate::line::Line2;

#[derive(Clone, Default, Debug, Serialize, Deserialize, SerializeHierarchy)]
pub struct LineData {
    pub lines_in_robot: Vec<Line2>,
    pub used_vertical_filtered_segments:
        HashSet<Point2<u16>, BuildHasherDefault<fxhash::FxHasher64>>,
}

#[derive(Clone, Debug, Serialize, Deserialize, SerializeHierarchy)]
pub enum LineDiscardReason {
    TooFewPoints,
    LineTooShort,
    LineTooLong,
    TooFarAway,
}

#[derive(Clone, Default, Debug, Serialize, Deserialize, SerializeHierarchy)]
pub struct ImageLines {
    pub discarded_lines: Vec<(Line2, LineDiscardReason)>,
    pub lines: Vec<Line2>,
    pub points: Vec<Point2<f32>>,
}
