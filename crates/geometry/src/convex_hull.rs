use std::{
    iter,
    ops::{Index, Mul},
};

use itertools::Itertools;
use linear_algebra::{Isometry2, Point2, Vector2};
use nalgebra::Matrix2;

use crate::{direction::Direction, line_segment::LineSegment};

pub enum Range {
    Full,
    OnlyBottomHalf,
}

pub fn convex_hull_gift_wrapping<Frame>(
    points: &[Point2<Frame>],
    range: Range,
) -> Vec<Point2<Frame>>
where
    Frame: Copy,
{
    // https://en.wikipedia.org/wiki/Gift_wrapping_algorithm
    // Modification: This implementation iterates from left to right until a smaller x value is found
    if points.is_empty() {
        return vec![];
    }
    let mut point_on_hull = *points
        .iter()
        .min_by(|a, b| a.x().total_cmp(&b.x()))
        .unwrap();
    let mut convex_hull = vec![];
    loop {
        convex_hull.push(point_on_hull);
        let mut candidate_end_point = points[0];
        for point in points.iter() {
            let last_point_on_hull_to_candidate_end_point = candidate_end_point - point_on_hull;
            let last_point_on_hull_to_point = *point - point_on_hull;
            let determinant = Matrix2::from_columns(&[
                last_point_on_hull_to_candidate_end_point.inner,
                last_point_on_hull_to_point.inner,
            ])
            .determinant();
            let point_is_left_of_candidate_end_point = determinant < 0.0;
            if candidate_end_point == point_on_hull || point_is_left_of_candidate_end_point {
                candidate_end_point = *point;
            }
        }
        // begin of modification
        if matches!(range, Range::OnlyBottomHalf) && candidate_end_point.x() < point_on_hull.x() {
            break;
        }
        // end of modification
        point_on_hull = candidate_end_point;
        if candidate_end_point == *convex_hull.first().unwrap() {
            break;
        }
    }
    convex_hull
}

#[derive(Clone, Debug)]
pub struct ConvexHull<Frame> {
    vertices: Vec<Point2<Frame>>,
}

impl<Frame> ConvexHull<Frame> {
    pub fn edges(&self) -> impl '_ + Iterator<Item = (Point2<Frame>, Point2<Frame>)> {
        self.vertices
            .iter()
            .cloned()
            .cycle()
            .tuple_windows()
            .take(self.vertices.len())
    }

    pub fn midpoint(&self) -> Point2<Frame> {
        (self
            .vertices
            .iter()
            .map(|x| x.coords())
            .sum::<Vector2<Frame>>()
            * 1.0
            / self.vertices.len() as f32)
            .as_point()
    }

    pub fn merge(
        &self,
        other: &ConvexHull<Frame>,
    ) -> Result<ConvexHull<Frame>, ConvexHullMergeError> {
        let midpoint_1 = self.midpoint();
        let midpoint_2 = other.midpoint();

        let (start1, _) = self
            .edges()
            .enumerate()
            .find(|(_index, (a, b))| {
                LineSegment(*a, *b).intersects_line_segment(LineSegment(midpoint_1, midpoint_2))
            })
            .ok_or(ConvexHullMergeError::MidpointContainedInOtherHull)?;
        let (start2, _) = other
            .edges()
            .enumerate()
            .find(|(_index, (a, b))| {
                LineSegment(*a, *b).intersects_line_segment(LineSegment(midpoint_1, midpoint_2))
            })
            .ok_or(ConvexHullMergeError::MidpointContainedInOtherHull)?;

        let mut i = start1 as isize;
        let mut j = start2 as isize;

        loop {
            println!("{i}, {j}");
            if LineSegment(self[i], other[j]).get_direction(self[i + 1])
                == Direction::Counterclockwise
            {
                i += 1;
                continue;
            }
            if LineSegment(self[i], other[j]).get_direction(other[j - 1])
                == Direction::Counterclockwise
            {
                j -= 1;
                continue;
            }
            break;
        }

        let range_1_start = i.rem_euclid(self.vertices.len() as isize) as usize;
        let range_2_end = j.rem_euclid(other.vertices.len() as isize) as usize;

        let mut i = start1 as isize;
        let mut j = start2 as isize;

        loop {
            println!("{i}, {j}");
            if LineSegment(self[i], other[j]).get_direction(self[i - 1]) == Direction::Clockwise {
                i -= 1;
                continue;
            }
            if LineSegment(self[i], other[j]).get_direction(other[j + 1]) == Direction::Clockwise {
                j += 1;
                continue;
            }
            break;
        }

        let range_1_end = i.rem_euclid(self.vertices.len() as isize) as usize;
        let range_2_start = j.rem_euclid(other.vertices.len() as isize) as usize;

        println!("{range_1_start}..{range_1_end}");
        println!("{range_2_start}..{range_2_end}");

        let self_vertices: Vec<_> = if range_1_end >= range_1_start {
            (range_1_start..=range_1_end).collect()
        } else {
            (range_1_start..self.vertices.len())
                .chain(0..=range_1_end)
                .collect()
        };
        let other_vertices: Vec<_> = if range_2_end >= range_2_start {
            (range_2_start..=range_2_end).collect()
        } else {
            (range_2_start..other.vertices.len())
                .chain(0..=range_2_end)
                .collect()
        };
        dbg!(&self_vertices);
        dbg!(&other_vertices);

        let vertices = self_vertices
            .into_iter()
            .map(|index| self.vertices[index])
            .chain(
                other_vertices
                    .into_iter()
                    .map(|index| other.vertices[index]),
            )
            .collect();

        Ok(ConvexHull { vertices })
    }
}

impl<Frame> Index<isize> for ConvexHull<Frame> {
    type Output = Point2<Frame>;

    fn index(&self, index: isize) -> &Self::Output {
        &self.vertices[index.rem_euclid(self.vertices.len() as isize) as usize]
    }
}
impl<Frame> Index<usize> for ConvexHull<Frame> {
    type Output = Point2<Frame>;

    fn index(&self, index: usize) -> &Self::Output {
        &self.vertices[index.rem_euclid(self.vertices.len())]
    }
}

impl<From, To> Mul<ConvexHull<From>> for Isometry2<From, To> {
    type Output = ConvexHull<To>;

    fn mul(self, rhs: ConvexHull<From>) -> Self::Output {
        ConvexHull {
            vertices: rhs.vertices.iter().map(|vertex| self * vertex).collect(),
        }
    }
}

#[derive(Debug)]
pub enum ConvexHullMergeError {
    MidpointContainedInOtherHull,
}

#[cfg(test)]
mod test {
    use super::*;
    use coordinate_systems::Ground;
    use linear_algebra::{point, vector};

    #[test]
    fn test_convex_hull() {
        let hexagon = vec![
            point![-1.0, 0.0],
            point![-0.5, -0.86],
            point![0.5, -0.86],
            point![1.0, 0.0],
            point![0.5, 0.86],
            point![-0.5, 0.86],
        ];
        let bottom_half_hexagon = vec![
            point![-1.0, 0.0],
            point![-0.5, -0.86],
            point![0.5, -0.86],
            point![1.0, 0.0],
        ];
        assert_eq!(
            hexagon,
            convex_hull_gift_wrapping::<Ground>(&hexagon, Range::Full)
        );
        assert_eq!(
            bottom_half_hexagon,
            convex_hull_gift_wrapping::<Ground>(&hexagon, Range::OnlyBottomHalf)
        );
    }

    #[test]
    fn merge_convex_hulls_hexagon() {
        #[derive(Debug, Clone, Copy)]
        struct Frame;
        let a = ConvexHull::<Frame> {
            vertices: vec![
                point![-1.0, 0.0],
                point![-0.5, -0.86],
                point![0.5, -0.86],
                point![1.0, 0.0],
                point![0.5, 0.86],
                point![-0.5, 0.86],
            ],
        };
        let b = Isometry2::from_parts(vector![2.0, 0.5], 0.0) * a.clone();

        let result = a.merge(&b);

        dbg!(&result);
        assert_eq!(result.unwrap().vertices.len(), 8);
        panic!()
    }

    #[test]
    fn merge_convex_hulls_square() {
        #[derive(Debug, Clone, Copy)]
        struct Frame;
        let a = ConvexHull::<Frame> {
            vertices: vec![
                point![0.0, 0.0],
                point![-1.0, 0.0],
                point![-1.0, -1.0],
                point![0.0, -1.0],
            ],
        };

        let b = Isometry2::from_parts(vector![2.0, 2.0], 0.0) * a.clone();

        let result = a.merge(&b).unwrap();
        let result_expensive = convex_hull_gift_wrapping(
            &a.vertices
                .iter()
                .chain(&b.vertices)
                .cloned()
                .collect::<Vec<_>>(),
            Range::Full,
        );

        dbg!(&result);
        dbg!(&result_expensive);

        for v in &result_expensive {
            assert!(result.vertices.contains(v));
        }
        for v in &result.vertices {
            assert!(result_expensive.contains(v));
        }

        assert_eq!(result.vertices.len(), 6);
        panic!()
    }
}
