#[tokio::main]
async fn main() {
  let opts = Options::parse();
  let mut cp = match opts.ai_checkpoint {
    Some(path) => crashpilot::CrashPilot::with_ai_checkpoint(path).await,
    None => crashpilot::CrashPilot::default().await,
  };

  cp.run().await
}

#[derive(Default)]
struct Options {
  ai_checkpoint: Option<String>,
}

impl Options {
  fn parse() -> Self {
    let mut opts = Self::default();
    let mut args = std::env::args().skip(1);

    while let Some(arg) = args.next() {
      match arg.as_str() {
        "--ai-checkpoint" | "--ai-model" => {
          opts.ai_checkpoint = Some(
            args
              .next()
              .unwrap_or_else(|| panic!("{arg} requires a path argument")),
          );
        }
        "--help" | "-h" => {
          print_help();
          std::process::exit(0);
        }
        other => panic!("unknown argument {other}; run with --help for usage"),
      }
    }

    opts
  }
}

fn print_help() {
  println!(
    "Usage: crashpilot [--ai-checkpoint <path>]\n\n\
     <path> may be a model.safetensors file, a checkpoint directory, or a run directory containing checkpoint_* directories.\n\n\
     Environment fallback for embedded callers: CRASHPILOT_AI_CHECKPOINT=<path>"
  );
}
