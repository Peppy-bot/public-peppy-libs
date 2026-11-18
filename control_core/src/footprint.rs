//! A mobile base's footprint: the outline it covers about its center, in
//! meters in the base's frame, as base_state:v1's describe_base gives it.
//!
//! An outline is three or more points, each once, counterclockwise, whose
//! edges, the last closing back to the first, do not cross one another, and
//! which encloses the base's center, the frame's origin, off its edges.
//! [`Footprint::parse`] holds a footprint to that rule, and
//! [`Footprint::points`] gives its points back as parsed, the points
//! describe_base carries.

/// The outline a base covers about its center, in meters in the base's frame:
/// three or more points, counterclockwise, as the module states.
#[derive(Debug, Clone, PartialEq)]
pub struct Footprint {
    points: Vec<[f64; 2]>,
}

/// Why points make no footprint.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum FootprintError {
    #[error("a footprint of {0} points is no polygon: give at least 3 points")]
    TooFew(usize),
    #[error(
        "footprint point {index}, {point:?}, is not a point: give finite coordinates in meters"
    )]
    NotFinite { index: usize, point: [f64; 2] },
    #[error("footprint points {first} and {second} are one point: give each point once")]
    RepeatedPoint { first: usize, second: usize },
    #[error(
        "the footprint's edges from point {first} and from point {second} cross: give an \
         outline that does not cross itself"
    )]
    Crossing { first: usize, second: usize },
    #[error(
        "the footprint's points run clockwise or enclose no area: give them counterclockwise \
         about the base's center"
    )]
    NotCounterclockwise,
    #[error(
        "the footprint leaves the base's center, the robot frame's origin, out or on its \
         outline: give an outline around the origin"
    )]
    CenterOutside,
}

impl Footprint {
    /// The footprint of `points`, in meters in the base's frame.
    pub fn parse(points: &[[f64; 2]]) -> Result<Self, FootprintError> {
        check_outline(points)?;
        Ok(Self {
            points: points.to_vec(),
        })
    }

    /// The outline's points, counterclockwise, as parsed.
    pub fn points(&self) -> &[[f64; 2]] {
        &self.points
    }
}

/// Checks that `points` make a footprint: three or more finite points, each
/// once, counterclockwise, an outline that does not cross itself and
/// encloses the origin off its edges.
fn check_outline(points: &[[f64; 2]]) -> Result<(), FootprintError> {
    if points.len() < 3 {
        return Err(FootprintError::TooFew(points.len()));
    }
    if let Some((index, &point)) = points
        .iter()
        .enumerate()
        .find(|(_, point)| !point.iter().all(|c| c.is_finite()))
    {
        return Err(FootprintError::NotFinite { index, point });
    }
    let n = points.len();
    let pairs = || (0..n).flat_map(move |first| (first + 1..n).map(move |second| (first, second)));
    if let Some((first, second)) = pairs().find(|&(a, b)| points[a] == points[b]) {
        return Err(FootprintError::RepeatedPoint { first, second });
    }
    let edge = |i: usize| (points[i], points[(i + 1) % n]);
    if let Some((first, second)) = pairs().find(|&(a, b)| edges_cross(edge(a), edge(b), n, a, b)) {
        return Err(FootprintError::Crossing { first, second });
    }
    let twice_area: f64 = (0..n)
        .map(|i| {
            let (a, b) = edge(i);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    if twice_area <= 0.0 {
        return Err(FootprintError::NotCounterclockwise);
    }
    if !encloses_origin(points) {
        return Err(FootprintError::CenterOutside);
    }
    Ok(())
}

/// Twice the signed area of the triangle `a`, `b`, `c`: above 0 when `c` lies
/// left of the line from `a` to `b`.
fn turn(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// Whether `p`, on the line through `a` and `b`, lies on the segment between
/// them.
fn within(a: [f64; 2], b: [f64; 2], p: [f64; 2]) -> bool {
    a[0].min(b[0]) <= p[0]
        && p[0] <= a[0].max(b[0])
        && a[1].min(b[1]) <= p[1]
        && p[1] <= a[1].max(b[1])
}

/// Whether two edges of an outline of `n` points, from points `i` and `j`,
/// `i` before `j`, meet anywhere but at the vertex two neighboring edges
/// share.
fn edges_cross(
    (a, b): ([f64; 2], [f64; 2]),
    (c, d): ([f64; 2], [f64; 2]),
    n: usize,
    i: usize,
    j: usize,
) -> bool {
    let neighbors = j == i + 1 || (i == 0 && j == n - 1);
    if neighbors {
        // The shared vertex is b == c, or d == a for the closing edge: the
        // edges overlap when they run back along one line.
        let (before, shared, after) = if j == i + 1 { (a, b, d) } else { (c, a, b) };
        return turn(before, shared, after) == 0.0
            && (within(before, shared, after) || within(shared, after, before));
    }
    let (t1, t2) = (turn(c, d, a), turn(c, d, b));
    let (t3, t4) = (turn(a, b, c), turn(a, b, d));
    if t1 * t2 < 0.0 && t3 * t4 < 0.0 {
        return true;
    }
    (t1 == 0.0 && within(c, d, a))
        || (t2 == 0.0 && within(c, d, b))
        || (t3 == 0.0 && within(a, b, c))
        || (t4 == 0.0 && within(a, b, d))
}

/// Whether the origin lies inside the outline `points`, off every edge: by
/// the crossings of a ray along +x.
fn encloses_origin(points: &[[f64; 2]]) -> bool {
    let n = points.len();
    let on_an_edge = (0..n).any(|i| {
        let (a, b) = (points[i], points[(i + 1) % n]);
        turn(a, b, [0.0, 0.0]) == 0.0 && within(a, b, [0.0, 0.0])
    });
    let crossings = (0..n)
        .filter(|&i| {
            let (a, b) = (points[i], points[(i + 1) % n]);
            (a[1] > 0.0) != (b[1] > 0.0)
                && a[0] + (0.0 - a[1]) * (b[0] - a[0]) / (b[1] - a[1]) > 0.0
        })
        .count();
    !on_an_edge && crossings % 2 == 1
}

#[cfg(test)]
mod tests {
    use super::*;

    // A footprint holds each point once, does not cross itself, and encloses
    // the base's center off its outline; a concave outline around its center
    // is one.
    #[test]
    fn a_footprint_wants_a_simple_outline_around_its_center() {
        assert_eq!(
            Footprint::parse(&[[-0.2, -0.2], [0.2, -0.2], [0.2, 0.2], [0.2, -0.2]]),
            Err(FootprintError::RepeatedPoint {
                first: 1,
                second: 3
            })
        );
        // A bow tie: its second edge crosses its fourth.
        assert_eq!(
            Footprint::parse(&[[-0.2, -0.2], [0.2, 0.2], [0.2, -0.2], [-0.2, 0.2]]),
            Err(FootprintError::Crossing {
                first: 0,
                second: 2
            })
        );
        // A spike runs back along its own edge.
        assert!(matches!(
            Footprint::parse(&[
                [-0.2, -0.2],
                [0.4, -0.2],
                [0.2, -0.2],
                [0.2, 0.2],
                [-0.2, 0.2]
            ]),
            Err(FootprintError::Crossing { .. })
        ));
        assert_eq!(
            Footprint::parse(&[[0.1, -0.1], [0.5, -0.1], [0.5, 0.1], [0.1, 0.1]]),
            Err(FootprintError::CenterOutside)
        );
        assert_eq!(
            Footprint::parse(&[[0.0, -0.1], [0.5, -0.1], [0.5, 0.1], [0.0, 0.1]]),
            Err(FootprintError::CenterOutside)
        );
        let notched = [
            [-0.3, -0.3],
            [0.3, -0.3],
            [0.3, 0.3],
            [0.1, 0.3],
            [0.1, 0.1],
            [-0.1, 0.1],
            [-0.1, 0.3],
            [-0.3, 0.3],
        ];
        let parsed = Footprint::parse(&notched).expect("a notched square around its center");
        assert_eq!(parsed.points(), notched);
    }

    #[test]
    fn a_footprint_wants_three_finite_points_counterclockwise() {
        assert_eq!(
            Footprint::parse(&[[0.0, 0.0], [1.0, 0.0]]),
            Err(FootprintError::TooFew(2))
        );
        assert_eq!(
            Footprint::parse(&[[0.0, 0.0], [1.0, f64::INFINITY], [0.0, 1.0]]),
            Err(FootprintError::NotFinite {
                index: 1,
                point: [1.0, f64::INFINITY]
            })
        );
        assert_eq!(
            Footprint::parse(&[[-0.2, 0.3], [0.4, 0.3], [0.4, -0.3], [-0.2, -0.3]]),
            Err(FootprintError::NotCounterclockwise)
        );
    }
}
