use std::{
    convert::Into,
    sync::{mpsc, Arc},
    time::{Duration, SystemTime},
};

use buffered_watch::Receiver;
use color_eyre::{eyre::WrapErr, Result};

use control::localization::generate_initial_pose;
use framework::{future_queue, Producer, RecordingTrigger};
use linear_algebra::vector;
use parameters::directory::deserialize;
use projection::camera_matrix::CameraMatrix;
use spl_network_messages::{HulkMessage, JerseyNumber};
use types::{messages::IncomingMessage, motion_selection::MotionSafeExits};

use crate::{
    cyclers::control::{Cycler, CyclerInstance, Database},
    interfake::{FakeDataInterface, Interfake},
    structs::Parameters,
};

pub struct Robot {
    pub interface: Arc<Interfake>,
    pub database: Database,
    pub parameters: Parameters,
    pub is_penalized: bool,
    pub last_kick_time: Duration,
    pub ball_last_seen: Option<SystemTime>,

    cycler: Cycler<Interfake>,
    control_receiver: Receiver<Database>,
    spl_network_sender: Producer<crate::structs::spl_network::MainOutputs>,
}

impl Robot {
    pub fn try_new(jersey_number: JerseyNumber) -> Result<Self> {
        let runtime = tokio::runtime::Builder::new_current_thread()
            .build()
            .unwrap();
        let mut parameter: Parameters = runtime.block_on(async {
            deserialize(
                "etc/parameters",
                &format!("behavior_simulator.{}", from_jersey_number(jersey_number)),
                &format!("behavior_simulator.{}", from_jersey_number(jersey_number)),
            )
            .await
            .wrap_err("could not load initial parameters")
        })?;
        parameter.jersey_number = jersey_number;

        let interface: Arc<_> = Interfake::default().into();

        let (control_sender, control_receiver) = buffered_watch::channel(Database::default());
        let (mut subscriptions_sender, subscriptions_receiver) =
            buffered_watch::channel(Default::default());
        let (mut parameters_sender, parameters_receiver) =
            buffered_watch::channel(Default::default());
        let (spl_network_sender, spl_network_consumer) = future_queue();
        let (recording_sender, _recording_receiver) = mpsc::sync_channel(0);
        *parameters_sender.borrow_mut() = parameter.clone();

        let mut cycler = Cycler::new(
            CyclerInstance::Control,
            interface.clone(),
            control_sender,
            subscriptions_receiver,
            parameters_receiver,
            spl_network_consumer,
            recording_sender,
            RecordingTrigger::new(0),
        )?;
        cycler.cycler_state.motion_safe_exits = MotionSafeExits::fill(true);

        let mut database = Database::default();

        database.main_outputs.ground_to_field = Some(
            generate_initial_pose(
                &parameter.localization.initial_poses[jersey_number],
                &parameter.field_dimensions,
            )
            .as_transform(),
        );
        database.main_outputs.has_ground_contact = true;
        database.main_outputs.is_localization_converged = true;
        subscriptions_sender
            .borrow_mut()
            .insert("additional_outputs".to_string());

        Ok(Self {
            interface,
            database,
            parameters: parameter,
            is_penalized: false,
            last_kick_time: Duration::default(),
            ball_last_seen: None,

            cycler,
            control_receiver,
            spl_network_sender,
        })
    }

    pub fn cycle(&mut self, messages: &[(JerseyNumber, HulkMessage)]) -> Result<()> {
        for (source, hulks_message) in messages.iter() {
            let source_is_other = *source != self.parameters.jersey_number;
            let message = IncomingMessage::Spl(*hulks_message);
            self.spl_network_sender.announce();
            self.spl_network_sender
                .finalize(crate::structs::spl_network::MainOutputs {
                    filtered_message: source_is_other.then(|| message.clone()),
                    message,
                });
        }
        buffered_watch::Sender::<_>::borrow_mut(
            &mut self.interface.get_last_database_sender().lock(),
        )
        .main_outputs = self.database.main_outputs.clone();

        self.cycler.cycle()?;

        let database = self.control_receiver.borrow_and_mark_as_seen();
        self.database.main_outputs = database.main_outputs.clone();
        self.database.additional_outputs = database.additional_outputs.clone();
        Ok(())
    }

    pub fn field_of_view(&self) -> f32 {
        let image_size = vector![640.0, 480.0];
        let focal_lengths = self
            .parameters
            .camera_matrix_parameters
            .vision_top
            .focal_lengths;
        let focal_lengths_scaled = image_size.inner.cast().component_mul(&focal_lengths);
        let field_of_view = CameraMatrix::calculate_field_of_view(focal_lengths_scaled, image_size);

        field_of_view.x
    }
}

pub fn to_jersey_number(value: usize) -> Result<JerseyNumber, String> {
    let number = match value {
        1 => JerseyNumber::One,
        2 => JerseyNumber::Two,
        3 => JerseyNumber::Three,
        4 => JerseyNumber::Four,
        5 => JerseyNumber::Five,
        6 => JerseyNumber::Six,
        7 => JerseyNumber::Seven,
        number => return Err(format!("invalid player number: {number}")),
    };

    Ok(number)
}

pub fn from_jersey_number(val: JerseyNumber) -> usize {
    match val {
        JerseyNumber::One => 1,
        JerseyNumber::Two => 2,
        JerseyNumber::Three => 3,
        JerseyNumber::Four => 4,
        JerseyNumber::Five => 5,
        JerseyNumber::Six => 6,
        JerseyNumber::Seven => 7,
    }
}
