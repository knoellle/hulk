use nalgebra::Point2;
use serde::{Deserialize, Serialize};
use serialize_hierarchy::SerializeHierarchy;

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub enum ObstacleKind {
    Ball,
    GoalPost,
    Robot,
    Unknown,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, SerializeHierarchy)]
pub struct Obstacle {
    #[leaf]
    pub kind: ObstacleKind,
    pub position: Point2<f32>,
    pub radius_at_foot_height: f32,
    pub radius_at_hip_height: f32,
}

impl Obstacle {
    pub fn ball(position: Point2<f32>, radius: f32) -> Self {
        Self {
            kind: ObstacleKind::Ball,
            position,
            radius_at_foot_height: radius,
            radius_at_hip_height: radius,
        }
    }

    pub fn robot(
        position: Point2<f32>,
        radius_at_foot_height: f32,
        radius_at_hip_height: f32,
    ) -> Self {
        Self {
            kind: ObstacleKind::Robot,
            position,
            radius_at_foot_height,
            radius_at_hip_height,
        }
    }

    pub fn goal_post(position: Point2<f32>, radius: f32) -> Self {
        Self {
            kind: ObstacleKind::GoalPost,
            position,
            radius_at_foot_height: radius,
            radius_at_hip_height: radius,
        }
    }
}
