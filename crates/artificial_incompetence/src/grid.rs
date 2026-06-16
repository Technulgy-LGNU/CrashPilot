use core_dump::vec::types::Vec2;
use tch::{Device, Tensor};

pub const GRID_W: i64 = 12;
pub const GRID_H: i64 = 12;

#[derive(Debug, Clone, Copy)]
pub struct GridSpec {
  pub cols: i64,
  pub rows: i64,
  pub min: Vec2<f32>,
  pub max: Vec2<f32>,
}

impl GridSpec {
  pub fn new(cols: i64, rows: i64, min: Vec2<f32>, max: Vec2<f32>) -> Self {
    Self {
      cols,
      rows,
      min,
      max,
    }
  }

  pub fn default_ssl() -> Self {
    Self {
      cols: GRID_W,
      rows: GRID_H,
      min: Vec2 { x: -4.5, y: -3.0 },
      max: Vec2 { x: 4.5, y: 3.0 },
    }
  }

  pub fn num_zones(&self) -> i64 {
    self.cols * self.rows
  }

  pub fn zone_center(&self, idx: i64) -> Vec2<f32> {
    let col = idx % self.cols;
    let row = idx / self.cols;

    let cell_w = (self.max.x - self.min.x) / self.cols as f32;
    let cell_h = (self.max.y - self.min.y) / self.rows as f32;

    Vec2 {
      x: self.min.x + (col as f32 + 0.5) * cell_w,
      y: self.min.y + (row as f32 + 0.5) * cell_h,
    }
  }

  pub fn zone_features(&self, device: Device) -> Tensor {
    let mut data = Vec::with_capacity((self.num_zones() * 2) as usize);
    for row in 0..self.rows {
      for col in 0..self.cols {
        let col_norm = if self.cols > 1 {
          col as f32 / (self.cols - 1) as f32
        } else {
          0.0
        };
        let row_norm = if self.rows > 1 {
          row as f32 / (self.rows - 1) as f32
        } else {
          0.0
        };
        data.push(col_norm);
        data.push(row_norm);
      }
    }
    Tensor::from_slice(&data)
      .view([self.num_zones(), 2])
      .to_device(device)
  }
}
