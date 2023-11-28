use clap::Parser;
use color_eyre::eyre::{Result, WrapErr};
use hula_types::RobotState;
use log::{info, LevelFilter};
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
    let mut file = File::open("/home/maik/lola_to_hula_passthrough.2023_11_28_12_33_32")
        .wrap_err("Failed to open file")?;
    //let csv_file = File::create("kneepitch.csv").wrap_err("Could not create csv file")?;
    //csv_file.write_all(b"timestamp,leftkneepitch,rightkneepitch\n")?;
    loop {
        //info!("{:?}", &lola_message);
        let mut timestamp_buf = [0; 16];
        file.read_exact(&mut timestamp_buf)
            .wrap_err("Could not read next timestamp")?;
        let timestamp = u128::from_be_bytes(timestamp_buf);

        info!("{}", timestamp);
        let robot_state = read_lola_message(&mut file).wrap_err("failed to read lola message")?;
        //serde serialize to csv or json, polars dataframe (pandas in cool und in rust)
        /*
        let stiffness = robot_state.stiffness.into_lola();
        let positions = robot_state.position.into_lola();
        let temperature = robot_state.temperature.into_lola();
        let current = robot_state.current.into_lola();
        let battery_charge = robot_state.battery.charge;
        let battery_current = robot_state.battery.current;
        let battery_temperature = robot_state.battery.temperature;
        let accelerometer_x = robot_state.inertial_measurement_unit.accelerometer.x;
        let accelerometer_y = robot_state.inertial_measurement_unit.accelerometer.y;
        let accelerometer_z = robot_state.inertial_measurement_unit.accelerometer.z;
        let imu_angles_x = robot_state.inertial_measurement_unit.angles.x;
        let imu_angles_y = robot_state.inertial_measurement_unit.angles.y;
        let gyroscope_x = robot_state.inertial_measurement_unit.gyroscope.x;
        let gyroscope_y = robot_state.inertial_measurement_unit.gyroscope.y;
        let gyroscope_z = robot_state.inertial_measurement_unit.gyroscope.z;
        let fsr = robot_state.force_sensitive_resistors;
        */

        //csv_file
        //  .write_all(line.as_bytes())
        //.wrap_err("Could not write to file")?;
    }
    Ok(())
}
fn read_lola_message(lola: &mut File) -> Result<RobotState> {
    let mut lola_data = [0; LOLA_BUFF_SIZE];
    lola.read_exact(&mut lola_data)
        .wrap_err("failed to read next message pack")?;
    from_slice(&lola_data).wrap_err("failed to parse MessagePack from LoLA StateMessage")
}
