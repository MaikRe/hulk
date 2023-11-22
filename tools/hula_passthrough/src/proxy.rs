use color_eyre::eyre::{bail, Result, WrapErr};
use epoll::{ControlOptions, Event, Events};
use log::{debug, error, info};
use std::{
    collections::HashMap,
    io::{BufWriter, Read, Write},
    os::unix::{io::AsRawFd, net::UnixStream, prelude::RawFd},
    thread::sleep,
    time::{Duration, Instant},
};
const LOLA_SOCKET_PATH: &str = "/tmp/robocup";
const LOLA_SOCKET_RETRY_COUNT: usize = 60;
const LOLA_SOCKET_RETRY_INTERVAL: Duration = Duration::from_secs(1);
const NO_EPOLL_TIMEOUT: i32 = -1;

fn wait_for_lola() -> Result<UnixStream> {
    for _ in 0..LOLA_SOCKET_RETRY_COUNT {
        if let Ok(socket) = UnixStream::connect(LOLA_SOCKET_PATH) {
            return Ok(socket);
        }
        info!("Waiting for LoLA socket to become available...");
        sleep(LOLA_SOCKET_RETRY_INTERVAL);
    }
    bail!("stopped after {} retries", LOLA_SOCKET_RETRY_COUNT)
}

pub struct Proxy {
    lola: UnixStream,
    epoll_fd: RawFd,
}

impl Proxy {
    pub fn initialize() -> Result<Self> {
        let lola = wait_for_lola().wrap_err("failed to connect to LoLA")?;
        let epoll_fd = epoll::create(false).wrap_err("failed to create epoll file descriptor")?;
        add_to_epoll(epoll_fd, lola.as_raw_fd())
            .wrap_err("failed to register LoLA file descriptor in epoll")?;

        Ok(Self { lola, epoll_fd })
    }
    pub fn run(mut self) -> Result<()> {
        let proxy_start = Instant::now();
        let mut connections = HashMap::new();
        let mut events = [Event::new(Events::empty(), 0); 16];
        let mut writer = BufWriter::with_capacity(786, self.lola.try_clone()?);
        debug!("Entering epoll loop...");
        loop {
            let number_of_events = epoll::wait(self.epoll_fd, NO_EPOLL_TIMEOUT, &mut events)
                .wrap_err("failed to wait for epoll")?;

            for event in &events[0..number_of_events] {
                let notified_fd = event.data as i32;
                if notified_fd == self.lola.as_raw_fd() {
                    debug!("Found lola event");
                    handle_lola_event(&mut self.lola, &mut connections)?;
                    /*
                    handle_lola_event(
                        &mut self.lola,
                        &mut connections,
                        proxy_start,
                        &self.shared_state,
                    )?;
                    */
                }
                /*lse if notified_fd == self.hula.as_raw_fd() {
                    register_connection(&mut self.hula, &mut connections, self.epoll_fd)?;
                } else {
                    handle_connection_event(
                        &mut connections,
                        notified_fd,
                        &mut writer,
                        &self.shared_state,
                    )?;
                }
                */
            }
            /*
            if !connections
                .values()
                .any(|connection| connection.is_sending_control_frames)
            {
                let battery = self.shared_state.lock().unwrap().battery;
                send_idle(&mut writer, battery).wrap_err(
                    "a shadowy flight into the dangerous world of a man who does not exist",
                )?;
            }
            */
        }
    }
}
struct Connection {
    socket: UnixStream,
    is_sending_control_frames: bool,
}
fn handle_lola_event(
    lola: &mut UnixStream,
    connections: &mut HashMap<RawFd, Connection>,
) -> Result<()> {
    let mut lola_data = [0; 896];
    lola.read_exact(&mut lola_data)
        .wrap_err("failed to read from LoLA socket")?;
    if connections.is_empty() {
        return Ok(());
    }
    connections.retain(|_, connection| {
        if let Err(error) = connection.socket.flush() {
            error!("Failed to flush connection: {error}");
            return false;
        }
        true
    });
    Ok(())
}
fn add_to_epoll(
    poll_file_descriptor: RawFd,
    file_descriptor_to_add: RawFd,
) -> Result<(), systemd::Error> {
    epoll::ctl(
        poll_file_descriptor,
        ControlOptions::EPOLL_CTL_ADD,
        file_descriptor_to_add,
        Event::new(
            Events::EPOLLIN | Events::EPOLLERR | Events::EPOLLHUP,
            file_descriptor_to_add as u64,
        ),
    )
}
