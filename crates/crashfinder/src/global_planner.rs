use std::cmp::Ordering;
use std::collections::BinaryHeap;

use core_dump::vec::types::Vec2;

use crate::geometry::{
  EPSILON, closest_point_on_segment, is_finite, lerp, nearest_point_outside_rectangle,
  point_in_rectangle, segment_distance, segment_intersects_rectangle,
};
use crate::types::{
  CrashfinderConfig, CrashfinderRequest, DebugGraphEdge, DebugGraphNode, DebugObstacle,
  DebugObstacleKind, DebugObstacleShape,
};

#[derive(Debug, Clone)]
pub(crate) struct PlanningWorld {
  pub bounds_min: Vec2<f32>,
  pub bounds_max: Vec2<f32>,
  obstacles: Vec<WorldObstacle>,
}

#[derive(Debug, Clone, Copy)]
struct WorldObstacle {
  debug: DebugObstacle,
  ignored_by_global_search: bool,
}

impl PlanningWorld {
  pub fn from_request(
    request: &CrashfinderRequest,
    config: &CrashfinderConfig,
    robot_position: Vec2<f32>,
  ) -> Option<Self> {
    let static_margin = config.robot_radius_mm + config.static_clearance_mm;
    let half_width = request.field_data.width * 0.5 + request.field_data.runoff_area;
    let half_height = request.field_data.height * 0.5 + request.field_data.runoff_area;
    let bounds_min = Vec2::new(-half_width + static_margin, -half_height + static_margin);
    let bounds_max = Vec2::new(half_width - static_margin, half_height - static_margin);
    if bounds_min.x >= bounds_max.x || bounds_min.y >= bounds_max.y {
      return None;
    }

    let mut obstacles = Vec::with_capacity(request.robots.len() + 3);
    if request.avoid_penalty() {
      let penalty_half_height = request.field_data.penalty_area_height * 0.5;
      let field_half_width = request.field_data.width * 0.5;
      let left_min = Vec2::new(
        -field_half_width - static_margin,
        -penalty_half_height - static_margin,
      );
      let left_max = Vec2::new(
        -field_half_width + request.field_data.penalty_area_width + static_margin,
        penalty_half_height + static_margin,
      );
      let right_min = Vec2::new(
        field_half_width - request.field_data.penalty_area_width - static_margin,
        -penalty_half_height - static_margin,
      );
      let right_max = Vec2::new(
        field_half_width + static_margin,
        penalty_half_height + static_margin,
      );

      for (min, max) in [(left_min, left_max), (right_min, right_max)] {
        let debug = DebugObstacle {
          kind: DebugObstacleKind::PenaltyArea,
          shape: DebugObstacleShape::Rectangle { min, max },
        };
        obstacles.push(WorldObstacle {
          debug,
          // If avoidance is enabled while a robot is already inside, let the
          // global route leave the area. Local tracking still chooses the
          // shortest route toward the requested target.
          ignored_by_global_search: point_in_rectangle(robot_position, min, max),
        });
      }
    }

    let robot_obstacle_radius = config.robot_radius_mm * 2.0 + config.robot_clearance_mm;
    for robot in &request.robots {
      if robot.robot_id == request.robot_id && robot.team == request.robot_team {
        continue;
      }
      if !is_finite(robot.pos) {
        continue;
      }

      let velocity = robot
        .vel
        .filter(|velocity| is_finite(*velocity))
        .unwrap_or_default();
      let predicted = robot.pos + velocity * config.global_prediction_horizon_seconds;
      let shape = DebugObstacleShape::Capsule {
        start: robot.pos,
        end: predicted,
        radius_mm: robot_obstacle_radius,
      };
      obstacles.push(WorldObstacle {
        debug: DebugObstacle {
          kind: DebugObstacleKind::Robot {
            team: robot.team,
            id: robot.robot_id,
          },
          shape,
        },
        // A global grid cannot resolve an overlap at its starting node. ORCA
        // handles that collision until the next global replan.
        ignored_by_global_search: shape_contains(shape, robot_position),
      });
    }

    if request.avoid_ball()
      && request.avoidance_zone > 0.0
      && let Some(ball) = request.ball
      && is_finite(ball.pos)
    {
      obstacles.push(WorldObstacle {
        debug: DebugObstacle {
          kind: DebugObstacleKind::Ball,
          shape: DebugObstacleShape::Circle {
            center: ball.pos,
            radius_mm: request.avoidance_zone,
          },
        },
        ignored_by_global_search: false,
      });
    }

    Some(Self {
      bounds_min,
      bounds_max,
      obstacles,
    })
  }

  pub fn debug_obstacles(&self) -> Vec<DebugObstacle> {
    self
      .obstacles
      .iter()
      .map(|obstacle| obstacle.debug)
      .collect()
  }

  pub fn point_is_free(&self, point: Vec2<f32>) -> bool {
    if point.x < self.bounds_min.x
      || point.x > self.bounds_max.x
      || point.y < self.bounds_min.y
      || point.y > self.bounds_max.y
    {
      return false;
    }

    self
      .obstacles
      .iter()
      .filter(|obstacle| !obstacle.ignored_by_global_search)
      .all(|obstacle| !shape_contains(obstacle.debug.shape, point))
  }

  pub fn segment_is_free(&self, start: Vec2<f32>, end: Vec2<f32>) -> bool {
    if !self.point_is_free(start) || !self.point_is_free(end) {
      return false;
    }
    self
      .obstacles
      .iter()
      .filter(|obstacle| !obstacle.ignored_by_global_search)
      .all(|obstacle| !shape_intersects_segment(obstacle.debug.shape, start, end))
  }

  pub fn segment_is_rule_free(&self, start: Vec2<f32>, end: Vec2<f32>) -> bool {
    let inside_bounds = |point: Vec2<f32>| {
      point.x >= self.bounds_min.x
        && point.x <= self.bounds_max.x
        && point.y >= self.bounds_min.y
        && point.y <= self.bounds_max.y
    };
    inside_bounds(start)
      && inside_bounds(end)
      && self
        .obstacles
        .iter()
        .filter(|obstacle| {
          !obstacle.ignored_by_global_search
            && obstacle.debug.kind == DebugObstacleKind::PenaltyArea
        })
        .all(|obstacle| !shape_intersects_segment(obstacle.debug.shape, start, end))
  }

  /// Projects rule-constrained targets to the closest legal point. Robots are
  /// deliberately not projected around because they are transient obstacles.
  pub fn constrain_target(&self, requested: Vec2<f32>, robot_position: Vec2<f32>) -> Vec2<f32> {
    let mut target = Vec2::new(
      requested.x.clamp(self.bounds_min.x, self.bounds_max.x),
      requested.y.clamp(self.bounds_min.y, self.bounds_max.y),
    );

    for _ in 0..4 {
      let previous = target;
      for obstacle in &self.obstacles {
        match (obstacle.debug.kind, obstacle.debug.shape) {
          (DebugObstacleKind::PenaltyArea, DebugObstacleShape::Rectangle { min, max })
            if point_in_rectangle(target, min, max) =>
          {
            let boundary = nearest_point_outside_rectangle(target, min, max);
            let mut outward = boundary - Vec2::new((min.x + max.x) * 0.5, (min.y + max.y) * 0.5);
            if outward.norm_squared() <= EPSILON * EPSILON {
              outward = robot_position - boundary;
            }
            target = boundary + outward.normalized();
          }
          (DebugObstacleKind::Ball, DebugObstacleShape::Circle { center, radius_mm })
            if target.distance(center) < radius_mm =>
          {
            let mut outward = target - center;
            if outward.norm_squared() <= EPSILON * EPSILON {
              outward = robot_position - center;
            }
            if outward.norm_squared() <= EPSILON * EPSILON {
              outward = Vec2::new(1.0, 0.0);
            }
            target = center + outward.normalized() * radius_mm;
          }
          _ => {}
        }
      }
      target.x = target.x.clamp(self.bounds_min.x, self.bounds_max.x);
      target.y = target.y.clamp(self.bounds_min.y, self.bounds_max.y);
      if target.distance(previous) <= EPSILON {
        break;
      }
    }
    target
  }
}

fn shape_contains(shape: DebugObstacleShape, point: Vec2<f32>) -> bool {
  const CLEARANCE_TOLERANCE_MM: f32 = 1.0e-3;
  match shape {
    DebugObstacleShape::Circle { center, radius_mm } => {
      point.distance(center) + CLEARANCE_TOLERANCE_MM < radius_mm
    }
    DebugObstacleShape::Capsule {
      start,
      end,
      radius_mm,
    } => point.distance_to_segment(start, end) + CLEARANCE_TOLERANCE_MM < radius_mm,
    DebugObstacleShape::Rectangle { min, max } => point_in_rectangle(point, min, max),
  }
}

fn shape_intersects_segment(shape: DebugObstacleShape, start: Vec2<f32>, end: Vec2<f32>) -> bool {
  const CLEARANCE_TOLERANCE_MM: f32 = 1.0e-3;
  match shape {
    DebugObstacleShape::Circle { center, radius_mm } => {
      center.distance_to_segment(start, end) + CLEARANCE_TOLERANCE_MM < radius_mm
    }
    DebugObstacleShape::Capsule {
      start: capsule_start,
      end: capsule_end,
      radius_mm,
    } => {
      segment_distance(start, end, capsule_start, capsule_end) + CLEARANCE_TOLERANCE_MM < radius_mm
    }
    DebugObstacleShape::Rectangle { min, max } => {
      segment_intersects_rectangle(start, end, min, max)
    }
  }
}

#[derive(Debug, Clone, Default)]
pub(crate) struct GlobalPlan {
  pub raw_path: Vec<Vec2<f32>>,
  pub smoothed_path: Vec<Vec2<f32>>,
  pub search_nodes: Vec<DebugGraphNode>,
  pub search_edges: Vec<DebugGraphEdge>,
}

impl GlobalPlan {
  pub fn found(&self) -> bool {
    !self.smoothed_path.is_empty()
  }
}

pub(crate) fn plan_path(
  world: &PlanningWorld,
  start: Vec2<f32>,
  goal: Vec2<f32>,
  config: &CrashfinderConfig,
) -> GlobalPlan {
  let mut plan = GlobalPlan::default();
  if !world.point_is_free(start) || !world.point_is_free(goal) {
    return plan;
  }

  if world.segment_is_free(start, goal) {
    plan.raw_path = deduplicate_path(vec![start, goal]);
    plan.smoothed_path = plan.raw_path.clone();
    plan.search_nodes.push(DebugGraphNode {
      position: start,
      cost_from_start: 0.0,
      estimated_total_cost: start.distance(goal),
    });
    if start.distance(goal) > EPSILON {
      plan.search_nodes.push(DebugGraphNode {
        position: goal,
        cost_from_start: start.distance(goal),
        estimated_total_cost: start.distance(goal),
      });
      plan.search_edges.push(DebugGraphEdge {
        from: start,
        to: goal,
      });
    }
    return plan;
  }

  let Some(grid) = Grid::new(world, config.grid_resolution_mm, config.max_search_nodes) else {
    return plan;
  };
  let Some(start_index) = grid.closest_visible_node(world, start) else {
    return plan;
  };
  let Some(goal_index) = grid.closest_visible_node(world, goal) else {
    return plan;
  };

  let node_count = grid.columns * grid.rows;
  let mut costs = vec![f32::INFINITY; node_count];
  let mut parents = vec![usize::MAX; node_count];
  let mut closed = vec![false; node_count];
  let mut open = BinaryHeap::new();
  costs[start_index] = 0.0;
  open.push(HeapEntry {
    index: start_index,
    cost: 0.0,
    estimated_total: grid
      .position(start_index)
      .distance(grid.position(goal_index)),
  });

  let mut expanded = 0usize;
  let mut found = false;
  while let Some(entry) = open.pop() {
    if closed[entry.index] || entry.cost > costs[entry.index] + EPSILON {
      continue;
    }
    closed[entry.index] = true;
    expanded += 1;

    let position = grid.position(entry.index);
    plan.search_nodes.push(DebugGraphNode {
      position,
      cost_from_start: entry.cost,
      estimated_total_cost: entry.estimated_total,
    });
    let parent = parents[entry.index];
    if parent != usize::MAX {
      plan.search_edges.push(DebugGraphEdge {
        from: grid.position(parent),
        to: position,
      });
    }

    if entry.index == goal_index {
      found = true;
      break;
    }
    if expanded >= config.max_search_nodes {
      break;
    }

    for neighbor in grid.neighbors(entry.index) {
      if closed[neighbor] || !grid.transition_is_free(world, entry.index, neighbor) {
        continue;
      }
      let next_cost = entry.cost + position.distance(grid.position(neighbor));
      if next_cost + EPSILON >= costs[neighbor] {
        continue;
      }
      costs[neighbor] = next_cost;
      parents[neighbor] = entry.index;
      let estimated_total = next_cost + grid.position(neighbor).distance(grid.position(goal_index));
      open.push(HeapEntry {
        index: neighbor,
        cost: next_cost,
        estimated_total,
      });
    }
  }

  if !found {
    return plan;
  }

  let mut indices = vec![goal_index];
  let mut cursor = goal_index;
  while cursor != start_index {
    cursor = parents[cursor];
    if cursor == usize::MAX {
      return GlobalPlan {
        search_nodes: plan.search_nodes,
        search_edges: plan.search_edges,
        ..GlobalPlan::default()
      };
    }
    indices.push(cursor);
  }
  indices.reverse();

  let mut raw_path = Vec::with_capacity(indices.len() + 2);
  raw_path.push(start);
  raw_path.extend(indices.into_iter().map(|index| grid.position(index)));
  raw_path.push(goal);
  plan.raw_path = deduplicate_path(raw_path);

  let shortened = shorten_path(world, &plan.raw_path);
  plan.smoothed_path = round_corners(world, &shortened, config);
  if plan.smoothed_path.is_empty() {
    plan.smoothed_path = shortened;
  }
  plan
}

#[derive(Debug, Clone, Copy)]
struct HeapEntry {
  index: usize,
  cost: f32,
  estimated_total: f32,
}

impl PartialEq for HeapEntry {
  fn eq(&self, other: &Self) -> bool {
    self.index == other.index && self.estimated_total.to_bits() == other.estimated_total.to_bits()
  }
}

impl Eq for HeapEntry {}

impl PartialOrd for HeapEntry {
  fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
    Some(self.cmp(other))
  }
}

impl Ord for HeapEntry {
  fn cmp(&self, other: &Self) -> Ordering {
    other
      .estimated_total
      .total_cmp(&self.estimated_total)
      .then_with(|| other.cost.total_cmp(&self.cost))
      .then_with(|| other.index.cmp(&self.index))
  }
}

struct Grid {
  min: Vec2<f32>,
  resolution: f32,
  columns: usize,
  rows: usize,
}

impl Grid {
  fn new(world: &PlanningWorld, resolution: f32, max_search_nodes: usize) -> Option<Self> {
    let columns =
      (((world.bounds_max.x - world.bounds_min.x) / resolution).floor() as usize).checked_add(1)?;
    let rows =
      (((world.bounds_max.y - world.bounds_min.y) / resolution).floor() as usize).checked_add(1)?;
    let cell_count = columns.checked_mul(rows)?;
    if columns < 2 || rows < 2 || cell_count > max_search_nodes.saturating_mul(8).max(1_024) {
      return None;
    }
    Some(Self {
      min: world.bounds_min,
      resolution,
      columns,
      rows,
    })
  }

  #[inline]
  fn position(&self, index: usize) -> Vec2<f32> {
    let x = index % self.columns;
    let y = index / self.columns;
    Vec2::new(
      self.min.x + x as f32 * self.resolution,
      self.min.y + y as f32 * self.resolution,
    )
  }

  fn closest_visible_node(&self, world: &PlanningWorld, point: Vec2<f32>) -> Option<usize> {
    (0..self.columns * self.rows)
      .filter(|index| {
        let position = self.position(*index);
        world.point_is_free(position) && world.segment_is_free(point, position)
      })
      .min_by(|left, right| {
        self
          .position(*left)
          .distance(point)
          .total_cmp(&self.position(*right).distance(point))
      })
  }

  fn neighbors(&self, index: usize) -> impl Iterator<Item = usize> + '_ {
    let x = index % self.columns;
    let y = index / self.columns;
    const OFFSETS: [(isize, isize); 8] = [
      (-1, -1),
      (0, -1),
      (1, -1),
      (-1, 0),
      (1, 0),
      (-1, 1),
      (0, 1),
      (1, 1),
    ];
    OFFSETS.into_iter().filter_map(move |(dx, dy)| {
      let next_x = x.checked_add_signed(dx)?;
      let next_y = y.checked_add_signed(dy)?;
      (next_x < self.columns && next_y < self.rows).then_some(next_y * self.columns + next_x)
    })
  }

  fn transition_is_free(&self, world: &PlanningWorld, from_index: usize, to_index: usize) -> bool {
    let from = self.position(from_index);
    let to = self.position(to_index);
    if !world.segment_is_free(from, to) {
      return false;
    }

    let from_x = from_index % self.columns;
    let from_y = from_index / self.columns;
    let to_x = to_index % self.columns;
    let to_y = to_index / self.columns;
    if from_x != to_x && from_y != to_y {
      let horizontal = from_y * self.columns + to_x;
      let vertical = to_y * self.columns + from_x;
      return world.point_is_free(self.position(horizontal))
        && world.point_is_free(self.position(vertical));
    }
    true
  }
}

fn deduplicate_path(path: Vec<Vec2<f32>>) -> Vec<Vec2<f32>> {
  let mut result = Vec::with_capacity(path.len());
  for point in path {
    if result
      .last()
      .is_none_or(|previous: &Vec2<f32>| previous.distance(point) > EPSILON)
    {
      result.push(point);
    }
  }
  result
}

fn shorten_path(world: &PlanningWorld, path: &[Vec2<f32>]) -> Vec<Vec2<f32>> {
  if path.len() <= 2 {
    return path.to_vec();
  }

  let mut shortened = Vec::with_capacity(path.len());
  let mut current = 0usize;
  shortened.push(path[0]);
  while current < path.len() - 1 {
    let mut next = path.len() - 1;
    while next > current + 1 && !world.segment_is_free(path[current], path[next]) {
      next -= 1;
    }
    shortened.push(path[next]);
    current = next;
  }
  shortened
}

fn round_corners(
  world: &PlanningWorld,
  path: &[Vec2<f32>],
  config: &CrashfinderConfig,
) -> Vec<Vec2<f32>> {
  if path.len() <= 2 || config.corner_radius_mm <= EPSILON {
    return path.to_vec();
  }

  let mut rounded = Vec::with_capacity(path.len() * config.smoothing_samples_per_corner);
  rounded.push(path[0]);
  for index in 1..path.len() - 1 {
    let previous = path[index - 1];
    let corner = path[index];
    let next = path[index + 1];
    let incoming = corner - previous;
    let outgoing = next - corner;
    let incoming_length = incoming.norm();
    let outgoing_length = outgoing.norm();
    if incoming_length <= EPSILON || outgoing_length <= EPSILON {
      continue;
    }

    let incoming_direction = incoming / incoming_length;
    let outgoing_direction = outgoing / outgoing_length;
    if incoming_direction.dot(&outgoing_direction) > 0.999 {
      continue;
    }
    let cut = config
      .corner_radius_mm
      .min(incoming_length * 0.4)
      .min(outgoing_length * 0.4);
    let entry = corner - incoming_direction * cut;
    let exit = corner + outgoing_direction * cut;

    let mut curve = Vec::with_capacity(config.smoothing_samples_per_corner + 1);
    curve.push(entry);
    for sample in 1..=config.smoothing_samples_per_corner {
      let t = sample as f32 / config.smoothing_samples_per_corner as f32;
      let first = lerp(entry, corner, t);
      let second = lerp(corner, exit, t);
      curve.push(lerp(first, second, t));
    }

    let connection_is_free = rounded
      .last()
      .is_some_and(|last| world.segment_is_free(*last, entry));
    let curve_is_free = curve
      .windows(2)
      .all(|segment| world.segment_is_free(segment[0], segment[1]));
    if connection_is_free && curve_is_free {
      rounded.extend(curve);
    } else {
      rounded.push(corner);
    }
  }
  rounded.push(*path.last().unwrap_or(&path[0]));
  deduplicate_path(rounded)
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct PathTracking {
  pub local_target: Vec2<f32>,
  pub remaining_distance: f32,
  pub curvature: f32,
}

pub(crate) fn track_path(
  path: &[Vec2<f32>],
  robot_position: Vec2<f32>,
  lookahead: f32,
  world: &PlanningWorld,
) -> Option<PathTracking> {
  let progress = closest_progress(path, robot_position)?;
  let path_remaining = path_length_from(path, progress.segment, progress.t);
  let remaining_distance = robot_position.distance(progress.point) + path_remaining;

  let mut target_distance = lookahead.min(path_remaining);
  let mut local_target = point_after_progress(path, progress.segment, progress.t, target_distance);
  for _ in 0..8 {
    if world.segment_is_free(robot_position, local_target) {
      break;
    }
    target_distance *= 0.65;
    local_target = point_after_progress(path, progress.segment, progress.t, target_distance);
  }
  if !world.segment_is_free(robot_position, local_target) {
    return None;
  }

  let midpoint = point_after_progress(path, progress.segment, progress.t, target_distance * 0.5);
  let curvature = three_point_curvature(progress.point, midpoint, local_target);
  Some(PathTracking {
    local_target,
    remaining_distance,
    curvature,
  })
}

pub(crate) fn path_is_clear(
  path: &[Vec2<f32>],
  robot_position: Vec2<f32>,
  world: &PlanningWorld,
) -> bool {
  let Some(progress) = closest_progress(path, robot_position) else {
    return false;
  };
  if !world.segment_is_free(robot_position, progress.point) {
    return false;
  }
  let segment_end = path[progress.segment + 1];
  if !world.segment_is_free(progress.point, segment_end) {
    return false;
  }
  path[progress.segment + 1..]
    .windows(2)
    .all(|segment| world.segment_is_free(segment[0], segment[1]))
}

#[derive(Clone, Copy)]
struct PathProgress {
  segment: usize,
  t: f32,
  point: Vec2<f32>,
}

fn closest_progress(path: &[Vec2<f32>], point: Vec2<f32>) -> Option<PathProgress> {
  if path.len() < 2 {
    return None;
  }
  path
    .windows(2)
    .enumerate()
    .map(|(segment, points)| {
      let projection = closest_point_on_segment(point, points[0], points[1]);
      let length = points[0].distance(points[1]);
      let t = if length <= EPSILON {
        0.0
      } else {
        points[0].distance(projection) / length
      };
      (
        point.distance(projection),
        PathProgress {
          segment,
          t,
          point: projection,
        },
      )
    })
    .min_by(|left, right| left.0.total_cmp(&right.0))
    .map(|(_, progress)| progress)
}

fn path_length_from(path: &[Vec2<f32>], segment: usize, t: f32) -> f32 {
  let first_remaining = path[segment].distance(path[segment + 1]) * (1.0 - t);
  first_remaining
    + path[segment + 1..]
      .windows(2)
      .map(|points| points[0].distance(points[1]))
      .sum::<f32>()
}

fn point_after_progress(
  path: &[Vec2<f32>],
  segment: usize,
  t: f32,
  mut distance: f32,
) -> Vec2<f32> {
  let mut current = lerp(path[segment], path[segment + 1], t);
  for end in &path[segment + 1..] {
    let length = current.distance(*end);
    if distance <= length || length <= EPSILON {
      return if length <= EPSILON {
        *end
      } else {
        lerp(current, *end, distance / length)
      };
    }
    distance -= length;
    current = *end;
  }
  *path.last().unwrap_or(&current)
}

fn three_point_curvature(a: Vec2<f32>, b: Vec2<f32>, c: Vec2<f32>) -> f32 {
  let ab = a.distance(b);
  let bc = b.distance(c);
  let ac = a.distance(c);
  let denominator = ab * bc * ac;
  if denominator <= EPSILON {
    0.0
  } else {
    2.0 * (b - a).det(&(c - a)).abs() / denominator
  }
}

#[cfg(test)]
mod tests {
  use core_dump::types::cp_types::{FieldData, Robot};

  use super::*;

  fn request() -> CrashfinderRequest {
    CrashfinderRequest {
      robot_id: 0,
      robot_team: 1,
      robots: Vec::new(),
      ball: None,
      field_data: FieldData {
        height: 6_000.0,
        width: 9_000.0,
        runoff_area: 300.0,
        goal_width: 1_000.0,
        penalty_area_width: 1_000.0,
        penalty_area_height: 2_000.0,
      },
      end_position: Vec2::zero(),
      target_orientation: 0.0,
      flags: 1 << 1,
      avoidance_zone: 500.0,
    }
  }

  #[test]
  fn direct_paths_do_not_expand_a_grid() {
    let config = CrashfinderConfig::default();
    let request = request();
    let start = Vec2::new(-2_000.0, 0.0);
    let goal = Vec2::new(2_000.0, 0.0);
    let world = PlanningWorld::from_request(&request, &config, start).unwrap();
    let plan = plan_path(&world, start, goal, &config);
    assert_eq!(plan.raw_path, vec![start, goal]);
    assert_eq!(plan.search_edges.len(), 1);
  }

  #[test]
  fn target_inside_ball_zone_is_projected_to_its_boundary() {
    let config = CrashfinderConfig::default();
    let mut request = request();
    request.flags = 1 | (1 << 1);
    request.ball = Some(core_dump::types::cp_types::Ball {
      pos: Vec2::zero(),
      vel: None,
    });
    let start = Vec2::new(-2_000.0, 0.0);
    let world = PlanningWorld::from_request(&request, &config, start).unwrap();
    let target = world.constrain_target(Vec2::zero(), start);
    assert!((target.distance(Vec2::zero()) - request.avoidance_zone).abs() < 0.01);
  }

  #[test]
  fn smoothed_paths_remain_collision_free_across_varied_scenes() {
    let config = CrashfinderConfig::default();
    let start = Vec2::new(-3_500.0, -1_500.0);
    let goal = Vec2::new(3_500.0, 1_500.0);
    let mut seed = 0x5eed_u32;
    let mut routes_found = 0;

    for _ in 0..32 {
      let mut request = request();
      for id in 1..=8 {
        let mut position = Vec2::new(
          random_between(&mut seed, -2_700.0, 2_700.0),
          random_between(&mut seed, -2_200.0, 2_200.0),
        );
        if position.distance(start) < 500.0 || position.distance(goal) < 500.0 {
          position.y = -position.y;
        }
        request.robots.push(Robot {
          robot_id: id,
          team: if id % 2 == 0 { 1 } else { 2 },
          pos: position,
          vel: Some(Vec2::new(
            random_between(&mut seed, -500.0, 500.0),
            random_between(&mut seed, -500.0, 500.0),
          )),
          ..Robot::default()
        });
      }

      let world = PlanningWorld::from_request(&request, &config, start).unwrap();
      let plan = plan_path(&world, start, goal, &config);
      if !plan.found() {
        continue;
      }
      routes_found += 1;
      assert_eq!(plan.smoothed_path.first(), Some(&start));
      assert_eq!(plan.smoothed_path.last(), Some(&goal));
      assert!(
        plan
          .smoothed_path
          .windows(2)
          .all(|segment| world.segment_is_free(segment[0], segment[1]))
      );
    }

    assert!(routes_found >= 24);
  }

  fn random_between(seed: &mut u32, min: f32, max: f32) -> f32 {
    *seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
    let unit = (*seed >> 8) as f32 / (u32::MAX >> 8) as f32;
    min + (max - min) * unit
  }
}
