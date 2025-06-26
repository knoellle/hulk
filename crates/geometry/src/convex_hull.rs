use std::{
    fmt::Debug,
    mem::swap,
    ops::{Index, Mul},
};

use itertools::Itertools;
use linear_algebra::{vector, Isometry2, Orientation2, Point2, Rotation2, Vector2};
use nalgebra::Matrix2;

use crate::{
    direction::Direction,
    line::Line,
    line_segment::{signed_acute_angle, LineSegment},
};

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
    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }

    pub fn edges(&self) -> impl '_ + Iterator<Item = LineSegment<Frame>> + Clone {
        self.vertices
            .iter()
            .cloned()
            .cycle()
            .tuple_windows()
            .map(|(a, b)| LineSegment(a, b))
            .take(self.len())
    }

    pub fn midpoint(&self) -> Point2<Frame> {
        (self
            .vertices
            .iter()
            .map(|x| x.coords())
            .sum::<Vector2<Frame>>()
            * 1.0
            / self.len() as f32)
            .as_point()
    }

    pub fn merge_calipers(
        &self,
        other: &ConvexHull<Frame>,
    ) -> Result<ConvexHull<Frame>, ConvexHullMergeError> {
        let (leftmost_own, _) = self
            .vertices
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.x().total_cmp(&b.1.x()))
            .unwrap();
        let (leftmost_other, _) = other
            .vertices
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.x().total_cmp(&b.1.x()))
            .unwrap();

        let i = leftmost_own;
        let j = leftmost_other;

        let mut vertices = vec![];
        let mut direction = vector![0.0, -1.0];

        let edges_self = self
            .edges()
            .cycle()
            .skip(leftmost_own)
            .take(self.len())
            .enumerate()
            .peekable();
        let edges_other = other
            .edges()
            .cycle()
            .skip(leftmost_other)
            .take(other.len())
            .enumerate()
            .peekable();

        let self_is_outside = self[i].x() < other[j].x();
        let (mut inner, mut outer) = if self_is_outside {
            (edges_other, edges_self)
        } else {
            (edges_self, edges_other)
        };

        for _ in 0..15 {
            // if vertices.len() > 30 {
            //     panic!();
            // }
            let Some((o, outer_edge)) = outer.peek().cloned() else {
                break;
            };
            let Some((i, inner_edge)) = inner.peek().cloned() else {
                break;
            };
            println!("{o}, {i}");

            let next_outer_angle = direction.angle(&outer_edge.as_line().direction);
            let next_inner_angle = direction.angle(&inner_edge.as_line().direction);
            let next_crossing_angle = direction.angle(&(inner_edge.0 - outer_edge.0));

            if next_crossing_angle < next_outer_angle.min(next_inner_angle) {
                vertices.push(inner_edge.0);
                swap(&mut outer, &mut inner);
                println!("swap");
                continue;
            }
            if next_inner_angle < next_outer_angle {
                inner.next();
                direction = inner_edge.as_line().direction;
                continue;
            }

            // if direction.angle(&inner_edge.as_line().direction)
            //     <= direction.angle(&outer_edge.as_line().direction)
            // {
            //     inner.next();
            //     direction = inner_edge.as_line().direction;
            //     continue;
            // };
            // if outer_edge.get_direction(inner_edge.0) != Direction::Counterclockwise {
            //     vertices.push(inner_edge.0);
            //     swap(&mut outer, &mut inner);
            //     println!("swap");
            //     continue;
            // }

            outer.next();
            vertices.push(outer_edge.1);
            direction = outer_edge.as_line().direction;
            println!(
                "{}",
                Orientation2::from_vector(direction).angle().to_degrees()
            );
        }

        for remaining_outer_edge in outer {
            vertices.push(remaining_outer_edge.1 .1);
        }

        return Ok(Self { vertices });

        // 'outer: while let Some(outer_edge) = outer.next() {
        //     if vertices.len() > 30 {
        //         panic!();
        //     }
        //
        //     while let Some(inner_edge) = inner.peek() {
        //         if direction.angle(&inner_edge.as_line().direction)
        //             > direction.angle(&outer_edge.as_line().direction)
        //         {
        //             break;
        //         }
        //
        //         println!("skip {}", inner_edge.0.inner);
        //         direction = inner.next().unwrap().as_line().direction;
        //     }
        //     if let Some(inner_edge) = inner.peek() {
        //         if outer_edge.as_line().get_direction(inner_edge.0) != Direction::Counterclockwise {
        //             direction = inner_edge.0 - outer_edge.0;
        //             println!("{}", direction.as_point().inner);
        //             swap(&mut outer, &mut inner);
        //             println!("swap");
        //             continue 'outer;
        //         }
        //     }
        //
        //     direction = outer_edge.as_line().direction;
        //     println!("{}", direction.as_point().inner);
        //     vertices.push(outer_edge.0);
        //
        //     println!("push {}", outer_edge.0.inner);
        // }
        //
        // return Ok(Self { vertices });
        //
        // while i < self.len() && j < other.len() {
        //     // if self_is_outside {
        //     //     vertices.push(self[i]);
        //     // } else {
        //     //     vertices.push(other[i]);
        //     // }
        //
        //     if direction.angle(&(self[i + 1] - self[i]))
        //         < direction.angle(&(other[j + 1] - other[j]))
        //     {
        //         direction = self[i + 1] - self[i];
        //         i += 1;
        //     } else {
        //         direction = other[j + 1] - other[j];
        //         j += 1;
        //     }
        //
        //     let new_self_is_outside =
        //         Line::new(self[i], direction).get_direction(other[j]) != Direction::Clockwise;
        //     if new_self_is_outside != self_is_outside {
        //         if new_self_is_outside {
        //             dbg!(i);
        //             vertices.push(self[i]);
        //         } else {
        //             dbg!(j);
        //             vertices.push(other[i]);
        //         }
        //     }
        //     self_is_outside = new_self_is_outside;
        // }
        //
        // Ok(Self { vertices })
    }

    pub fn merge(
        &self,
        other: &ConvexHull<Frame>,
    ) -> Result<ConvexHull<Frame>, ConvexHullMergeError> {
        let midpoint_1 = dbg!(self.midpoint());
        let midpoint_2 = dbg!(other.midpoint());
        let connecting_line = LineSegment(midpoint_1, midpoint_2);

        let (start1, _) = self
            .edges()
            .enumerate()
            .find(|(_index, edge)| edge.intersects_line_segment(connecting_line))
            .ok_or(ConvexHullMergeError::MidpointContainedInOtherHull)?;
        let (start2, _) = other
            .edges()
            .enumerate()
            .find(|(_index, edge)| edge.intersects_line_segment(connecting_line))
            .ok_or(ConvexHullMergeError::MidpointContainedInOtherHull)?;

        let mut i = start1 as isize;
        let mut j = start2 as isize;

        let get_direction = |a: Point2<Frame>, b, c| LineSegment(a, b).get_direction(c);

        loop {
            println!("{i}, {j}");
            if get_direction(self[i], other[j], self[i + 1]) == Direction::Counterclockwise {
                i += 1;
                continue;
            }
            if get_direction(self[i], other[j], other[j - 1]) == Direction::Counterclockwise {
                j -= 1;
                continue;
            }
            break;
        }

        let range_1_start = i.rem_euclid(self.len() as isize) as usize;
        let range_2_end = j.rem_euclid(other.len() as isize) as usize;

        let mut i = start1 as isize;
        let mut j = start2 as isize;

        loop {
            println!("{i}, {j}");
            if get_direction(self[i], other[j], self[i - 1]) == Direction::Clockwise {
                i -= 1;
                continue;
            }
            if get_direction(self[i], other[j], other[j + 1]) == Direction::Clockwise {
                j += 1;
                continue;
            }
            break;
        }

        let range_1_end = i.rem_euclid(self.len() as isize) as usize;
        let range_2_start = j.rem_euclid(other.len() as isize) as usize;

        println!("{range_1_start}..{range_1_end}");
        println!("{range_2_start}..{range_2_end}");

        let self_vertices: Vec<_> = if range_1_end >= range_1_start {
            (range_1_start..=range_1_end).collect()
        } else {
            (range_1_start..self.len()).chain(0..=range_1_end).collect()
        };
        let other_vertices: Vec<_> = if range_2_end >= range_2_start {
            (range_2_start..=range_2_end).collect()
        } else {
            (range_2_start..other.len())
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
        &self.vertices[index.rem_euclid(self.len() as isize) as usize]
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

    use proptest::{proptest, strategy::Strategy};

    use coordinate_systems::Ground;
    use linear_algebra::{point, vector};

    #[derive(Debug, Clone, Copy)]
    struct Frame;

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

    // #[test]
    fn merge_convex_hulls_hexagon() {
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
        assert_eq!(result.unwrap().len(), 8);
        panic!()
    }

    // #[test]
    fn merge_convex_hulls_square() {
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

        assert_eq!(result.len(), 6);
        panic!()
    }

    proptest! {
        // #[test]
        fn random_polygons(
            vertices in proptest::collection::vec((-100.0..100.0, -100.0..100.0).prop_map(|(x, y)| -> Point2<Frame>  {point![x as f32, y as f32]}), 10..100),
            shift in (-100.0..100.0, -100.0..100.0).prop_map(|(x, y)| -> Vector2<Frame>  {vector![x as f32, y as f32]}),
        ) {
            let mut vertices_deduplicated = Vec::new();
            for v in vertices {
                if vertices_deduplicated.contains(&v) {
                    continue;
                }
                if v.x() == 0.0 || v.y() == 0.0 {
                    continue;
                }
                vertices_deduplicated.push(v);
            }
            let vertices_convex = convex_hull_gift_wrapping(&vertices_deduplicated, Range::Full);
            test_polygon(vertices_convex, shift);
        }
    }

    proptest! {
        #[test]
        fn hexagons(
            shift in (-5.0..5.0, -5.0..5.0).prop_map(|(x, y)| -> Vector2<Frame>  {vector![x as f32, y as f32]}),
        ) {
            let vertices= vec![
                point![-1.0, 0.0],
                point![-0.5, -0.86],
                point![0.5, -0.86],
                point![1.0, 0.0],
                point![0.5, 0.86],
                point![-0.5, 0.86],
            ];
            test_polygon(vertices, shift);
        }
    }

    #[test]
    fn hexagon_and_triangle() {
        let transform = Isometry2::from_parts(vector![-2.0, 1.0], 0.0);
        let vertices = vec![
            point![-1.0, 0.0],
            point![-0.5, -0.86],
            point![0.5, -0.86],
            point![1.0, 0.0],
            point![0.5, 0.86],
            point![-0.5, 0.86],
        ];
        let a = transform * ConvexHull::<Frame> { vertices };
        let vertices = vec![point![-0.5, -0.86], point![0.5, -0.86], point![0.0, 0.86]];
        let b = ConvexHull::<Frame> { vertices };

        let result = a.merge_calipers(&b).unwrap();
        let result_expensive = convex_hull_gift_wrapping(
            &a.vertices
                .iter()
                .chain(&b.vertices)
                .cloned()
                .collect::<Vec<_>>(),
            Range::Full,
        );

        dbg!(&result.vertices);
        dbg!(&result_expensive);

        assert_eq!(result.len(), result_expensive.len());

        for v in &result_expensive {
            assert!(result.vertices.contains(v));
        }
        for v in &result.vertices {
            assert!(result_expensive.contains(v));
        }
    }

    fn test_polygon(vertices: Vec<Point2<Frame>>, shift: Vector2<Frame>) {
        if shift.norm() < 0.01 {
            return;
        }
        if shift.x() == 0.0 || shift.y() == 0.0 {
            return;
        }

        dbg!(&vertices);

        let a = ConvexHull { vertices };

        let b = Isometry2::from_parts(shift, 0.0) * a.clone();

        let Ok(result) = a.merge_calipers(&b) else {
            return;
        };
        let result_expensive = convex_hull_gift_wrapping(
            &a.vertices
                .iter()
                .chain(&b.vertices)
                .cloned()
                .collect::<Vec<_>>(),
            Range::Full,
        );

        dbg!(&result.vertices);
        dbg!(&result_expensive);

        assert_eq!(result.len(), result_expensive.len());

        for v in &result_expensive {
            assert!(result.vertices.contains(v));
        }
        for v in &result.vertices {
            assert!(result_expensive.contains(v));
        }
    }
}
