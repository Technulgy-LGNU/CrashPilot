use artificial_incompetence::{
  evaluate_latest_checkpoint, train_all_stages, train_single_stage, EvaluationOptions,
  TrainOptions, TrainingStage,
};
use std::path::PathBuf;
use tch::Device;

fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
  let mut stage = None;
  let mut opts = TrainOptions::default();
  let mut eval_steps = 120usize;

  let mut args = std::env::args().skip(1);
  while let Some(arg) = args.next() {
    match arg.as_str() {
      "--stage" => stage = Some(parse_stage(&next_value(&mut args, "--stage")?)?),
      "--updates" => opts.updates = next_value(&mut args, "--updates")?.parse()?,
      "--worlds" => opts.worlds = next_value(&mut args, "--worlds")?.parse()?,
      "--rollout-steps" => {
        opts.rollout_steps = next_value(&mut args, "--rollout-steps")?.parse()?
      }
      "--checkpoint-every" => {
        opts.checkpoint_every = next_value(&mut args, "--checkpoint-every")?.parse()?
      }
      "--checkpoint-dir" => {
        opts.checkpoint_dir = PathBuf::from(next_value(&mut args, "--checkpoint-dir")?)
      }
      "--run-name" => opts.run_name = Some(next_value(&mut args, "--run-name")?),
      "--model" => opts.model_path = Some(PathBuf::from(next_value(&mut args, "--model")?)),
      "--lr" => opts.learning_rate = next_value(&mut args, "--lr")?.parse()?,
      "--eval-steps" => eval_steps = next_value(&mut args, "--eval-steps")?.parse()?,
      "--cuda" => opts.device = Device::Cuda(0),
      other => return Err(format!("unknown argument {other}").into()),
    }
  }

  let reports = if let Some(stage) = stage {
    vec![train_single_stage(stage, opts.clone())?]
  } else {
    train_all_stages(opts.clone())?
  };

  println!("training_reports={reports:#?}");

  for report in reports {
    if report.stage == TrainingStage::SumatraOpponent {
      continue;
    }
    let eval = evaluate_latest_checkpoint(
      report.stage,
      EvaluationOptions {
        checkpoint_dir: opts.checkpoint_dir.clone(),
        run_name: opts.run_name.as_ref().map(|run_name| {
          if stage.is_some() {
            run_name.clone()
          } else {
            format!("{run_name}/{}", report.stage.name())
          }
        }),
        device: opts.device,
        steps: eval_steps,
      },
    )?;
    println!("evaluation_report={eval:#?}");
  }

  Ok(())
}

fn next_value(
  args: &mut impl Iterator<Item = String>,
  name: &str,
) -> Result<String, Box<dyn std::error::Error + Send + Sync>> {
  args
    .next()
    .ok_or_else(|| format!("{name} requires a value").into())
}

fn parse_stage(stage: &str) -> Result<TrainingStage, Box<dyn std::error::Error + Send + Sync>> {
  match stage {
    "touch_ball" => Ok(TrainingStage::TouchBall),
    "dribble_to_goal" => Ok(TrainingStage::DribbleToGoal),
    "shoot_goal" => Ok(TrainingStage::ShootGoal),
    "pass_receive" => Ok(TrainingStage::PassReceive),
    "one_vs_one" => Ok(TrainingStage::OneVsOne),
    "scripted_scrimmage" => Ok(TrainingStage::ScriptedScrimmage),
    "sumatra_opponent" => Ok(TrainingStage::SumatraOpponent),
    other => Err(format!("unknown stage {other}").into()),
  }
}
