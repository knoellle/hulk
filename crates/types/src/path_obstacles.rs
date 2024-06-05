use std::collections::HashSet;

use geometry::{arc::Arc, circle::Circle, direction::Direction, line_segment::LineSegment};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

use coordinate_systems::Ground;

#[derive(
    Clone, Copy, Debug, Deserialize, Serialize, PathSerialize, PathDeserialize, PathIntrospect,
)]
pub enum PathObstacleShape {
    Circle(Circle<Ground>),
    /// Allows passing from the "left" of the line to the "right" but not the other way around.
    /// Left and right are identifed the winding direction of the triangle `[line.0, line.1, point]`.
    /// A polygon made of these lines with  vertices arrange counterclockwise can be passed out of
    /// but not into.
    OneWayLineSegment(LineSegment<Ground>),
}

impl PathObstacleShape {
    pub fn intersects_line_segment(&self, line_segment: LineSegment<Ground>) -> bool {
        match self {
            PathObstacleShape::Circle(circle) => circle.intersects_line_segment(&line_segment),
            PathObstacleShape::OneWayLineSegment(obstacle_line_segment) => {
                obstacle_line_segment.intersects_line_segment(line_segment)
                    && obstacle_line_segment.get_direction(line_segment.0) == Direction::Clockwise
            }
        }
    }

    pub fn overlaps_arc(&self, arc: Arc<Ground>, orientation: Direction) -> bool {
        match self {
            PathObstacleShape::Circle(circle) => circle.overlaps_arc(arc, orientation),
            PathObstacleShape::OneWayLineSegment(line_segment) => {
                let overlaps_arc = line_segment.overlaps_arc(arc, orientation);
                let two_crossings =
                    line_segment.get_direction(arc.start) != line_segment.get_direction(arc.end);
                let first_crossing_blocked =
                    line_segment.get_direction(arc.start) == Direction::Clockwise;

                overlaps_arc && (first_crossing_blocked || two_crossings)
            }
        }
    }

    pub fn as_circle(&self) -> Option<&Circle<Ground>> {
        match self {
            PathObstacleShape::Circle(circle) => Some(circle),
            _ => None,
        }
    }

    pub fn as_circle_mut(&mut self) -> Option<&mut Circle<Ground>> {
        match self {
            PathObstacleShape::Circle(circle) => Some(circle),
            _ => None,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize, PathSerialize, PathDeserialize, PathIntrospect)]
pub struct PathObstacle {
    pub shape: PathObstacleShape,
    pub nodes: Vec<usize>,
    pub populated_connections: HashSet<usize>,
}

impl From<PathObstacleShape> for PathObstacle {
    fn from(shape: PathObstacleShape) -> Self {
        Self {
            shape,
            nodes: vec![],
            populated_connections: HashSet::new(),
        }
    }
}

impl From<Circle<Ground>> for PathObstacle {
    fn from(shape: Circle<Ground>) -> Self {
        Self::from(PathObstacleShape::Circle(shape))
    }
}
impl From<LineSegment<Ground>> for PathObstacle {
    fn from(shape: LineSegment<Ground>) -> Self {
        Self::from(PathObstacleShape::OneWayLineSegment(shape))
    }
}
