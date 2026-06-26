use crate::grid::GridSpec;
use crate::modules::mlp::MLP;
use tch::nn::Module;
use tch::{Device, Kind, Tensor, nn};

pub struct GridZoneEncoder {
  zone_embed: nn::Embedding,
  coord_mlp: MLP,
  grid_spec: GridSpec,
  d_model: i64,
}

impl GridZoneEncoder {
  pub fn new(vs: &nn::Path, grid_spec: GridSpec, d_model: i64) -> Self {
    let zone_embed = nn::embedding(
      vs / "zone_embed",
      grid_spec.num_zones(),
      d_model,
      Default::default(),
    );
    let coord_mlp = MLP::new(&(vs / "coord_mlp"), 2, d_model, d_model);

    Self {
      zone_embed,
      coord_mlp,
      grid_spec,
      d_model,
    }
  }

  pub fn forward(&self, batch_size: i64, device: Device) -> Tensor {
    let ids = Tensor::arange(self.grid_spec.num_zones(), (Kind::Int64, device));
    let id_e = ids.apply(&self.zone_embed);
    let coord_e = self
      .coord_mlp
      .forward(&self.grid_spec.zone_features(device));
    let zones = id_e + coord_e;
    zones
      .unsqueeze(0)
      .expand([batch_size, self.grid_spec.num_zones(), self.d_model], true)
  }

  pub fn mask(&self, batch_size: i64, device: Device) -> Tensor {
    Tensor::ones(
      [batch_size, self.grid_spec.num_zones()],
      (Kind::Bool, device),
    )
  }
}
