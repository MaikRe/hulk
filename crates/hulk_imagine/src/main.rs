#![recursion_limit = "256"]
use std::time::SystemTime;
use std::{env::args, fs::File, path::PathBuf, sync::Arc};

use color_eyre::{
    eyre::{Result, WrapErr},
    install,
};
use framework::Parameters as FrameworkParameters;
use hardware::{
    ActuatorInterface, CameraInterface, IdInterface, MicrophoneInterface, NetworkInterface,
    PathsInterface, RecordingInterface, SensorInterface, SpeakerInterface, TimeInterface,
};
use types::{
    audio::SpeakerRequest,
    camera_position::CameraPosition,
    hardware::{Ids, Paths},
    joints::Joints,
    led::Leds,
    messages::{IncomingMessage, OutgoingMessage},
    samples::Samples,
    sensor_data::SensorData,
    ycbcr422_image::YCbCr422Image,
};

use crate::execution::ImageExtractor;

use ctrlc::set_handler;
use serde_json::from_reader;
use tokio_util::sync::CancellationToken;

pub trait HardwareInterface:
    ActuatorInterface
    + CameraInterface
    + IdInterface
    + MicrophoneInterface
    + NetworkInterface
    + PathsInterface
    + RecordingInterface
    + SensorInterface
    + SpeakerInterface
    + TimeInterface
{
}

include!(concat!(env!("OUT_DIR"), "/generated_code.rs"));

struct ReplayerHardwareInterface {
    ids: Ids,
}

impl ActuatorInterface for ReplayerHardwareInterface {
    fn write_to_actuators(
        &self,
        _positions: Joints<f32>,
        _stiffnesses: Joints<f32>,
        _leds: Leds,
    ) -> Result<()> {
        Ok(())
    }
}

impl CameraInterface for ReplayerHardwareInterface {
    fn read_from_camera(&self, _camera_position: CameraPosition) -> Result<YCbCr422Image> {
        panic!("Replayer cannot produce data from hardware")
    }
}

impl IdInterface for ReplayerHardwareInterface {
    fn get_ids(&self) -> Ids {
        self.ids.clone()
    }
}

impl MicrophoneInterface for ReplayerHardwareInterface {
    fn read_from_microphones(&self) -> Result<Samples> {
        panic!("Replayer cannot produce data from hardware")
    }
}

impl NetworkInterface for ReplayerHardwareInterface {
    fn read_from_network(&self) -> Result<IncomingMessage> {
        panic!("Replayer cannot produce data from hardware")
    }

    fn write_to_network(&self, _message: OutgoingMessage) -> Result<()> {
        Ok(())
    }
}

impl PathsInterface for ReplayerHardwareInterface {
    fn get_paths(&self) -> Paths {
        Paths {
            motions: "etc/motions".into(),
            neural_networks: "etc/neural_networks".into(),
            sounds: "etc/sounds".into(),
        }
    }
}

impl RecordingInterface for ReplayerHardwareInterface {
    fn should_record(&self) -> bool {
        false
    }

    fn set_whether_to_record(&self, _enable: bool) {}
}

impl SensorInterface for ReplayerHardwareInterface {
    fn read_from_sensors(&self) -> Result<SensorData> {
        panic!("Replayer cannot produce data from hardware")
    }
}

impl SpeakerInterface for ReplayerHardwareInterface {
    fn write_to_speakers(&self, _request: SpeakerRequest) {}
}

impl TimeInterface for ReplayerHardwareInterface {
    fn get_now(&self) -> SystemTime {
        SystemTime::now()
    }
}

impl HardwareInterface for ReplayerHardwareInterface {}

fn main() -> Result<()> {
    install()?;
    replayer()
}

pub fn replayer() -> Result<()> {
    let replay_path = PathBuf::from(
        args()
            .nth(1)
            .expect("expected replay path as first parameter"),
    );
    let framework_parameters_path = args()
        .nth(2)
        .unwrap_or("etc/parameters/framework.json".to_string());
    let keep_running = CancellationToken::new();
    set_handler({
        let keep_running = keep_running.clone();
        move || {
            keep_running.cancel();
        }
    })?;

    let file =
        File::open(framework_parameters_path).wrap_err("failed to open framework parameters")?;
    let framework_parameters: FrameworkParameters =
        from_reader(file).wrap_err("failed to parse framework parameters")?;

    let hardware_interface = ReplayerHardwareInterface {
        ids: Ids {
            body_id: "replayer".to_string(),
            head_id: "replayer".to_string(),
        },
    };

    let ids = hardware_interface.get_ids();

    let image_extractor = ImageExtractor::new(
        Arc::new(hardware_interface),
        replay_path.clone(),
        &ids.body_id,
        &ids.head_id,
        replay_path,
    )
    .wrap_err("failed to create replayer")?;

    Ok(())
}
