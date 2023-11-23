use chrono::Local;
use color_eyre::eyre::{bail, Result, WrapErr};
use epoll::{ControlOptions, Event, Events};
use log::{debug, error, info, warn};
use std::{
    collections::HashMap,
    fs::{remove_file, rename, File},
    io::{ErrorKind, Read, Write},
    os::unix::{
        io::AsRawFd,
        net::{UnixListener, UnixStream},
        prelude::RawFd,
    },
    thread::sleep,
    time::{Duration, Instant},
};
const LOLA_SOCKET_PATH: &str = "/tmp/robocup";
const LOLA_SOCKET_RETRY_COUNT: usize = 60;
const LOLA_SOCKET_RETRY_INTERVAL: Duration = Duration::from_secs(1);
const NO_EPOLL_TIMEOUT: i32 = -1;
const MOVED_SOCKET_PATH: &str = "/tmp/robocup_moved";
const BUFF_SIZE: usize = 896;

fn wait_for_lola() -> Result<UnixStream> {
    for _ in 0..LOLA_SOCKET_RETRY_COUNT {
        if let Ok(socket) = UnixStream::connect(LOLA_SOCKET_PATH) {
            return Ok(socket);
        }
        info!("Passthrough waiting for LoLA socket to become available...");
        sleep(LOLA_SOCKET_RETRY_INTERVAL);
    }
    bail!(
        "passthrough stopped after {} retries",
        LOLA_SOCKET_RETRY_COUNT
    )
}

pub struct Proxy {
    lola: UnixStream,
    hula_passthrough: UnixListener,
    epoll_fd: RawFd,
    lola_file: File,
    hula_file: File,
}

impl Proxy {
    pub fn initialize() -> Result<Self> {
        let lola = wait_for_lola().wrap_err("failed to connect to LoLA")?;
        rename(LOLA_SOCKET_PATH, MOVED_SOCKET_PATH).wrap_err("Failed to yoink robocup file")?;
        remove_file(LOLA_SOCKET_PATH)
            .or_else(|error| match error.kind() {
                ErrorKind::NotFound => Ok(()),
                _ => Err(error),
            })
            .wrap_err("passthrough failed to unlink existing HuLA socket file")?;

        let hula_passthrough = UnixListener::bind(LOLA_SOCKET_PATH)
            .wrap_err_with(|| format!("passthrough failed to bind {LOLA_SOCKET_PATH}"))?;

        let epoll_fd =
            epoll::create(false).wrap_err("passthrough failed to create epoll file descriptor")?;
        add_to_epoll(epoll_fd, lola.as_raw_fd())
            .wrap_err("passthrough failed to register LoLA file descriptor in epoll")?;
        add_to_epoll(epoll_fd, hula_passthrough.as_raw_fd())
            .wrap_err("passthrough failed to register hula file descriptor in epoll")?;
        let timestamp = Local::now().format("%Y_%m_%d_%H_%M_%S");
        let lola_file = File::create(format!("lola_to_hula_passthrough.{}", timestamp))
            .wrap_err("Failed to create log file of lola messages")?;
        let hula_file = File::create(format!("hula_to_lola_passthrough.{}", timestamp))
            .wrap_err("Failed to create log file of hula messages")?;
        Ok(Self {
            lola,
            hula_passthrough,
            epoll_fd,
            lola_file,
            hula_file,
        })
    }
    pub fn run(mut self) -> Result<()> {
        let proxy_start = Instant::now();
        let mut connections = HashMap::new();
        let mut events = [Event::new(Events::empty(), 0); 16];
        debug!("Entering epoll loop...");
        loop {
            let number_of_events = epoll::wait(self.epoll_fd, NO_EPOLL_TIMEOUT, &mut events)
                .wrap_err("failed to wait for epoll")?;
            for event in &events[0..number_of_events] {
                let notified_fd = event.data as i32;
                if notified_fd == self.lola.as_raw_fd() {
                    handle_lola_event(
                        &mut self.lola,
                        &mut connections,
                        proxy_start,
                        &mut self.lola_file,
                    )?;
                } else if notified_fd == self.hula_passthrough.as_raw_fd() {
                    register_connection(
                        &mut self.hula_passthrough,
                        &mut connections,
                        self.epoll_fd,
                    )?;
                } else {
                    handle_connection_event(
                        &mut connections,
                        notified_fd,
                        &mut self.lola,
                        proxy_start,
                        &mut self.hula_file,
                    )?;
                }
            }
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
    proxy_start: Instant,
    lola_file: &mut File,
) -> Result<()> {
    let since_start = format!("{:0>8}", proxy_start.elapsed().as_millis());
    let mut lola_data = [0; BUFF_SIZE];
    lola.read_exact(&mut lola_data)
        .wrap_err("failed to read from LoLA socket")?;
    lola_file
        .write_all(&since_start.as_bytes())
        .wrap_err("Could not write timestamp to lola file")?;
    lola_file
        .write_all(&mut lola_data)
        .wrap_err("Could not write data to lola file")?;
    if connections.is_empty() {
        debug!("Finished handling lola event due to no connections");
        return Ok(());
    }
    connections.retain(|_, connection| {
        if let Err(error) = connection.socket.write_all(&lola_data) {
            error!("Failed to write StateStorage to connection: {error}");
            return false;
        }
        if let Err(error) = connection.socket.flush() {
            error!("Failed to flush connection: {error}");
            return false;
        }
        debug!("Finished handling lola event through connection retain");
        true
    });
    debug!("Finished handling lola event");
    Ok(())
}

fn register_connection(
    hula: &mut UnixListener,
    connections: &mut HashMap<RawFd, Connection>,
    poll_fd: RawFd,
) -> Result<()> {
    let (connection_stream, _) = hula.accept().wrap_err("failed to accept connection")?;
    let connection_fd = connection_stream.as_raw_fd();
    info!("Accepted connection with file descriptor {connection_fd}");
    if connections
        .insert(
            connection_fd,
            Connection {
                socket: connection_stream,
                is_sending_control_frames: false,
            },
        )
        .is_some()
    {
        panic!("connection is already registered");
    }
    add_to_epoll(poll_fd, connection_fd)
        .wrap_err("failed to register connection file descriptor")?;
    Ok(())
}
fn handle_connection_event(
    connections: &mut HashMap<RawFd, Connection>,
    notified_fd: RawFd,
    lola: &mut UnixStream,
    proxy_start: Instant,
    hula_file: &mut File,
) -> Result<()> {
    match connections.get_mut(&notified_fd) {
        Some(connection) => {
            let mut read_buffer = [0; BUFF_SIZE];
            if let Err(error) = connection.socket.read_exact(&mut read_buffer) {
                error!("Failed to read from connection: {}", error);
                info!("Removing connection with file descriptor {}", notified_fd);
                // remove will drop, drop will close, close will EPOLL_CTL_DEL
                connections
                    .remove(&notified_fd)
                    .expect("connection file descriptor has to be registered");
                return Ok(());
            };
            lola.write_all(&mut read_buffer)
                .wrap_err("Could not forward message from hula to lola")?;
            let since_start = format!("{:0>8}", proxy_start.elapsed().as_millis());
            hula_file
                .write_all(since_start.as_bytes())
                .wrap_err("Could not write timestamp to hula file")?;
            hula_file
                .write_all(&mut read_buffer)
                .wrap_err("Could not write data to hula file")?;
            connection.is_sending_control_frames = true;
        }
        None => warn!(
            "Connection with file descriptor {} does not exist",
            notified_fd
        ),
    }
    debug!("Finished handling hula event");
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
