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
const HULA_BUFF_SIZE: usize = 786;

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
    hula_passthrough: UnixListener,
    epoll_fd: RawFd,
    connections: HashMap<RawFd, Connection>,
}

impl Proxy {
    pub fn initialize() -> Result<Self> {
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
        add_to_epoll(epoll_fd, hula_passthrough.as_raw_fd())
            .wrap_err("passthrough failed to register hula file descriptor in epoll")?;
        let timestamp = Local::now().format("%Y_%m_%d_%H_%M_%S");
        let mut connections = HashMap::new();
        Ok(Self {
            hula_passthrough,
            epoll_fd,
            connections,
        })
    }
    pub fn run(&mut self) -> Result<()> {
        let proxy_start = Instant::now();
        let mut events = [Event::new(Events::empty(), 0); 16];
        debug!("Entering epoll loop...");
        let mut connected = false;
        loop {
            let number_of_events = epoll::wait(self.epoll_fd, NO_EPOLL_TIMEOUT, &mut events)
                .wrap_err("failed to wait for epoll")?;
            let tik = proxy_start.elapsed().as_micros();
            for event in &events[0..number_of_events] {
                let notified_fd = event.data as i32;
                if notified_fd == self.hula_passthrough.as_raw_fd() {
                    register_connection(
                        &mut self.hula_passthrough,
                        &mut self.connections,
                        self.epoll_fd,
                    )?;
                    connected = true;
                } else {
                    handle_connection_event(&mut self.connections, notified_fd, proxy_start)?;
                }
            }
            if connected {
                info!("Broke loop");
                return Ok(());
            }
        }
    }
    pub fn test(mut self) {
        sleep(Duration::from_secs(5));
        let mut lola_data: [u8; 896] = [0; BUFF_SIZE];
        let test = "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB".as_bytes();
        let mut frame_now = Instant::now();
        let frame_length = Duration::from_millis(12);
        for _i in 0..10000 {
            if self.connections.is_empty() {
                debug!("Finished handling lola event due to no connections");
                return;
            }
            self.connections.retain(|_, connection| {
                if let Err(error) = connection.socket.write_all(&test) {
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
            frame_now += frame_length;
            sleep(frame_now - Instant::now());
        }
    }
}

struct Connection {
    socket: UnixStream,
}
fn handle_lola_event(
    lola: &mut UnixStream,
    connections: &mut HashMap<RawFd, Connection>,
    proxy_start: Instant,
) -> Result<()> {
    let since_start = proxy_start.elapsed().as_millis();
    let mut lola_data = [0; BUFF_SIZE];
    lola.read_exact(&mut lola_data)
        .wrap_err("failed to read from LoLA socket")?;
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
    proxy_start: Instant,
) -> Result<()> {
    match connections.get_mut(&notified_fd) {
        Some(connection) => {
            let mut read_buffer = [0; HULA_BUFF_SIZE];
            if let Err(error) = connection.socket.read(&mut read_buffer) {
                error!("Failed to read from connection: {}", error);
                info!("Removing connection with file descriptor {}", notified_fd);
                dbg!(read_buffer);
                // remove will drop, drop will close, close will EPOLL_CTL_DEL
                connections
                    .remove(&notified_fd)
                    .expect("connection file descriptor has to be registered");
                return Ok(());
            };
            let since_start = proxy_start.elapsed().as_millis();
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
