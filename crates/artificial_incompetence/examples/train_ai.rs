use artificial_incompetence::{
  EvaluationOptions, TrainOptions, TrainingStage, evaluate_latest_checkpoint,
  train_dribble_to_goal, train_one_vs_one, train_pass_receive, train_scripted_scrimmage,
  train_shoot_goal, train_sumatra_opponent, train_touch_ball,
};
use std::path::PathBuf;
use tch::Device;

fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
  let mut stage = TrainingStage::TouchBall;
  let mut opts = TrainOptions::default();
  let mut eval_steps = 120usize;

  let mut args = std::env::args().skip(1);
  while let Some(arg) = args.next() {
    match arg.as_str() {
      "--stage" => stage = parse_stage(&next_value(&mut args, "--stage")?)?,
      "--updates" => opts.updates = next_value(&mut args, "--updates")?.parse()?,
      "--worlds" => opts.worlds = next_value(&mut args, "--worlds")?.parse()?,
      "--rollout-steps" => opts.rollout_steps = next_value(&mut args, "--rollout-steps")?.parse()?,
      "--checkpoint-every" => {
        opts.checkpoint_every = next_value(&mut args, "--checkpoint-every")?.parse()?
      }
      "--checkpoint-dir" => opts.checkpoint_dir = PathBuf::from(next_value(&mut args, "--checkpoint-dir")?),
      "--run-name" => opts.run_name = Some(next_value(&mut args, "--run-name")?),
      "--model" => opts.model_path = Some(PathBuf::from(next_value(&mut args, "--model")?)),
      "--lr" => opts.learning_rate = next_value(&mut args, "--lr")?.parse()?,
      "--eval-steps" => eval_steps = next_value(&mut args, "--eval-steps")?.parse()?,
      "--cuda" => opts.device = Device::Cuda(0),
      other => return Err(format!("unknown argument {other}").into()),
    }
  }

  let report = match stage {
    TrainingStage::TouchBall => train_touch_ball(opts.clone())?,
    TrainingStage::DribbleToGoal => train_dribble_to_goal(opts.clone())?,
    TrainingStage::ShootGoal => train_shoot_goal(opts.clone())?,
    TrainingStage::PassReceive => train_pass_receive(opts.clone())?,
    TrainingStage::OneVsOne => train_one_vs_one(opts.clone())?,
    TrainingStage::ScriptedScrimmage => train_scripted_scrimmage(opts.clone())?,
    TrainingStage::SumatraOpponent => train_sumatra_opponent(opts.clone())?,
  };

  println!("training_report={report:#?}");

  if stage != TrainingStage::SumatraOpponent {
    let eval = evaluate_latest_checkpoint(
      stage,
      EvaluationOptions {
        checkpoint_dir: opts.checkpoint_dir,
        run_name: opts.run_name,
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
  args.next()
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
