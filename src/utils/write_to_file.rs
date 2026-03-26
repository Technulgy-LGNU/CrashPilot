use std::fs;
use std::io::ErrorKind;

pub async fn write_to_file<T>(path: &str, msg: T) where T: prost::Message + Default {
  let mut buf = Vec::new();
  match prost::Message::encode(&msg, &mut buf) {
    Ok(_) => {},
    Err(e) => eprintln!("Encode error: {}", e),
  };

  match fs::write(path, buf) {
    Ok(_) => (),
    Err(e) => match e.kind() {
      ErrorKind::InvalidFilename | ErrorKind::InvalidInput | ErrorKind::InvalidData => {
        eprintln!("Error while writing file due to invalid data: {:?}", e)
      },
      ErrorKind::PermissionDenied => {
        eprintln!("Error while writing file due to denied permissions");
      }
      _ => eprintln!("Error while writing file: {:?}", e),
    }
  }
}
