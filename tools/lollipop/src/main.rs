use clap::Parser;
use color_eyre::eyre::{Result, WrapErr};
use csv::Writer;
use flatten_json_object::Flattener;
use hula_types::RobotState;
use log::{debug, LevelFilter};
use rmp_serde::from_slice;
use std::{fs::File, io::Read};
const LOLA_BUFF_SIZE: usize = 896;
//const HULA_BUFF_SIZE: usize = 786;
#[derive(Parser, Debug)]
#[clap(
    name = "lollipop",
    about = "Open HULA/LoLA passthrough files and convert to csv"
)]
struct Arguments {
    /// Log with Debug log level
    #[arg(short, long)]
    verbose: bool,
}

fn main() -> Result<()> {
    let matches = Arguments::parse();
    env_logger::builder()
        .filter(
            None,
            if matches.verbose {
                LevelFilter::Debug
            } else {
                LevelFilter::Info
            },
        )
        .init();
    let mut filewriter = Writer::from_path("/home/maik/LOLADATA/lola30.csv")?;
    let mut file = File::open("/home/maik/LOLADATA/lola30").wrap_err("Failed to open file")?;
    //let mut csv_file = File::create("kneepitch.csv").wrap_err("Could not create csv file")?;
    //csv_file.write_all(b"timestamp,leftkneepitch,rightkneepitch\n")?;
    let mut timestamp_buf = [0; 16];
    let timestamp = u128::from_be_bytes(timestamp_buf);
    file.read_exact(&mut timestamp_buf)
        .wrap_err("Could not read first time stamp")?;
    let mut robot_state =
        read_lola_message(&mut file).wrap_err("Failed to read first lola message")?;
    robot_state.received_at = timestamp as f32;
    let jsonobject = serde_json::to_value(robot_state).wrap_err("Could not convert to json")?;
    let flattener = Flattener::new().set_key_separator(".");
    let output = flattener
        .flatten(&jsonobject)
        .wrap_err("Failed to flatten")?;
    let keys = output
        .as_object()
        .into_iter()
        .flatten()
        .map(|(key, _value)| (key.to_string()))
        .collect::<Vec<_>>();
    let values = output
        .as_object()
        .into_iter()
        .flatten()
        .map(|(_key, value)| (format!("{}", value.as_f64().unwrap())))
        .collect::<Vec<_>>();
    debug!("{:?}", keys);
    filewriter.write_record(keys)?;
    filewriter.write_record(values)?;
    //print!("{}", robot_state);
    while let Ok(()) = file.read_exact(&mut timestamp_buf) {
        let timestamp = u128::from_be_bytes(timestamp_buf);
        let mut robot_state =
            read_lola_message(&mut file).wrap_err("failed to read lola message")?;
        robot_state.received_at = timestamp as f32;
        let jsonobject = serde_json::to_value(robot_state).wrap_err("Could not convert to json")?;
        let output = flattener
            .flatten(&jsonobject)
            .wrap_err("Failed to flatten")?;
        let values = output
            .as_object()
            .into_iter()
            .flatten()
            .map(|(_key, value)| (format!("{}", value.as_f64().unwrap())))
            .collect::<Vec<_>>();
        filewriter.write_record(values)?;
    }

    Ok(())
}
fn read_lola_message(lola: &mut File) -> Result<RobotState> {
    let mut lola_data = [0; LOLA_BUFF_SIZE];
    lola.read_exact(&mut lola_data)
        .wrap_err("failed to read next message pack")?;
    from_slice(&lola_data).wrap_err("failed to parse MessagePack from LoLA StateMessage")
}
