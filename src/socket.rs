// socketcan/src/socket.rs
//
// Implements sockets for CANbus 2.0 and FD for SocketCAN on Linux.
//
// This file is part of the Rust 'socketcan-rs' library.
//
// Licensed under the MIT license:
//   <LICENSE or http://opensource.org/licenses/MIT>
// This file may not be copied, modified, or distributed except according
// to those terms.

//! Implementation of sockets for CANbus 2.0 and FD for SocketCAN on Linux.

use crate::{
    as_uninit_bytes, as_uninit_bytes_mut,
    frame::{can_frame_uninit, canfd_frame_uninit, AsPtr, CAN_ERR_MASK},
    CanAddr, CanAnyFrame, CanFdFrame, CanFrame, CanRawFrame, IoError, IoErrorKind, IoResult,
};
use libc::{can_frame, canid_t, socklen_t, EINPROGRESS};
use socket2::SockAddr;
use std::{
    fmt,
    io::{self, Read, Write},
    mem::{self, size_of, size_of_val, MaybeUninit},
    os::{
        raw::{c_int, c_void},
        unix::io::{AsFd, AsRawFd, BorrowedFd, IntoRawFd, OwnedFd, RawFd},
    },
    ptr,
    time::Duration,
};

pub use libc::{
    CANFD_MTU, CAN_MTU, CAN_RAW, CAN_RAW_ERR_FILTER, CAN_RAW_FD_FRAMES, CAN_RAW_FILTER,
    CAN_RAW_JOIN_FILTERS, CAN_RAW_LOOPBACK, CAN_RAW_RECV_OWN_MSGS, SOL_CAN_BASE, SOL_CAN_RAW,
};

/// Check an error return value for timeouts.
///
/// Due to the fact that timeouts are reported as errors, calling `read_frame`
/// on a socket with a timeout that does not receive a frame in time will
/// result in an error being returned. This trait adds a `should_retry` method
/// to `Error` and `Result` to check for this condition.
pub trait ShouldRetry {
    /// Check for timeout
    ///
    /// If `true`, the error is probably due to a timeout.
    fn should_retry(&self) -> bool;
}

impl ShouldRetry for IoError {
    fn should_retry(&self) -> bool {
        match self.kind() {
            // EAGAIN, EINPROGRESS and EWOULDBLOCK are the three possible codes
            // returned when a timeout occurs. the stdlib already maps EAGAIN
            // and EWOULDBLOCK os WouldBlock
            IoErrorKind::WouldBlock => true,
            // however, EINPROGRESS is also valid
            IoErrorKind::Other => {
                matches!(self.raw_os_error(), Some(errno) if errno == EINPROGRESS)
            }
            _ => false,
        }
    }
}

impl<E: fmt::Debug> ShouldRetry for IoResult<E> {
    fn should_retry(&self) -> bool {
        match *self {
            Err(ref e) => e.should_retry(),
            _ => false,
        }
    }
}

// ===== Private local helper functions =====

/// Tries to open the CAN socket by the interface number.
fn raw_open_socket(addr: &CanAddr, nonblocking: bool) -> IoResult<socket2::Socket> {
    let socket = rustix::net::socket_with(
        rustix::net::AddressFamily::CAN,
        rustix::net::SocketType::RAW,
        if nonblocking {
            rustix::net::SocketFlags::CLOEXEC | rustix::net::SocketFlags::NONBLOCK
        } else {
            rustix::net::SocketFlags::empty()
        },
        Some(rustix::net::Protocol::from_raw(
            (CAN_RAW as u32).try_into().unwrap(),
        )),
    )?;
    let sock = socket2::Socket::from(socket);
    sock.bind(&SockAddr::from(*addr))?;
    Ok(sock)
}

/// `setsockopt` wrapper
///
/// The libc `setsockopt` function is set to set various options on a socket.
/// `set_socket_option` offers a somewhat type-safe wrapper that does not
/// require messing around with `*const c_void`s.
///
/// A proper `std::io::Error` will be returned on failure.
///
/// Example use:
///
/// ```text
/// let fd = ...;  // some file descriptor, this will be stdout
/// set_socket_option(fd, SOL_TCP, TCP_NO_DELAY, 1 as c_int)
/// ```
///
/// Note that the `val` parameter must be specified correctly; if an option
/// expects an integer, it is advisable to pass in a `c_int`, not the default
/// of `i32`.
#[deprecated(since = "3.4.0", note = "Moved into `SocketOptions` trait")]
#[inline]
pub fn set_socket_option<T>(fd: c_int, level: c_int, name: c_int, val: &T) -> IoResult<()> {
    let ret = unsafe {
        libc::setsockopt(
            fd,
            level,
            name,
            val as *const _ as *const c_void,
            size_of::<T>() as socklen_t,
        )
    };

    match ret {
        0 => Ok(()),
        _ => Err(IoError::last_os_error()),
    }
}

/// Sets a collection of multiple socket options with one call.
#[deprecated(since = "3.4.0", note = "Moved into `SocketOptions` trait")]
pub fn set_socket_option_mult<T>(
    fd: c_int,
    level: c_int,
    name: c_int,
    values: &[T],
) -> IoResult<()> {
    let ret = if values.is_empty() {
        // can't pass in a ptr to a 0-len slice, pass a null ptr instead
        unsafe { libc::setsockopt(fd, level, name, ptr::null(), 0) }
    } else {
        unsafe {
            libc::setsockopt(
                fd,
                level,
                name,
                values.as_ptr().cast(),
                size_of_val(values) as socklen_t,
            )
        }
    };

    match ret {
        0 => Ok(()),
        _ => Err(IoError::last_os_error()),
    }
}

// ===== Common 'Socket' trait =====

/// Common trait for SocketCAN sockets.
///
/// Note that a socket it created by opening it, and then closed by
/// dropping it.
pub trait Socket: AsFd + AsRawFd {
    /// Open a named CAN device.
    ///
    /// Usually the more common case, opens a socket can device by name, such
    /// as "can0", "vcan0", or "socan0".
    fn open(ifname: &str) -> IoResult<Self>
    where
        Self: Sized,
    {
        Self::_open(ifname, false)
    }
    fn _open(ifname: &str, nonblocking: bool) -> IoResult<Self>
    where
        Self: Sized,
    {
        let addr = CanAddr::from_iface(ifname)?;
        Self::open_addr(&addr, nonblocking)
    }

    /// Open CAN device by interface number.
    ///
    /// Opens a CAN device by kernel interface number.
    fn open_iface(ifindex: u32, nonblocking: bool) -> IoResult<Self>
    where
        Self: Sized,
    {
        let addr = CanAddr::new(ifindex);
        Self::open_addr(&addr, nonblocking)
    }

    /// Open a CAN socket by address.
    fn open_addr(addr: &CanAddr, nonblocking: bool) -> IoResult<Self>
    where
        Self: Sized;

    /// The type of CAN frame that can be read and written by the socket.
    ///
    /// This is typically distinguished by the size of the supported frame,
    /// with the primary difference between a `CanFrame` and a `CanFdFrame`.
    type FrameType;

    /// Gets the read timout on the socket, if any.
    fn read_timeout(&self) -> IoResult<Option<Duration>> {
        Ok(rustix::net::sockopt::get_socket_timeout(
            self,
            rustix::net::sockopt::Timeout::Recv,
        )?)
    }

    /// Sets the read timeout on the socket
    ///
    /// For convenience, the result value can be checked using
    /// `ShouldRetry::should_retry` when a timeout is set.
    ///
    /// If the duration is set to `None` then write calls will block
    /// indefinitely.
    fn set_read_timeout<D>(&self, duration: D) -> IoResult<()>
    where
        D: Into<Option<Duration>>,
    {
        Ok(rustix::net::sockopt::set_socket_timeout(
            self,
            rustix::net::sockopt::Timeout::Recv,
            duration.into(),
        )?)
    }

    /// Gets the write timeout on the socket, if any.
    fn write_timeout(&self) -> IoResult<Option<Duration>> {
        Ok(rustix::net::sockopt::get_socket_timeout(
            self,
            rustix::net::sockopt::Timeout::Send,
        )?)
    }

    /// Sets the write timeout on the socket
    ///
    /// If the duration is set to `None` then write calls will block
    /// indefinitely.
    fn set_write_timeout<D>(&self, duration: D) -> IoResult<()>
    where
        D: Into<Option<Duration>>,
    {
        Ok(rustix::net::sockopt::set_socket_timeout(
            self,
            rustix::net::sockopt::Timeout::Send,
            duration.into(),
        )?)
    }

    /// Blocking read a single can frame.
    fn read_frame(&self) -> IoResult<Self::FrameType>;

    /// Blocking read a single can frame with timeout.
    fn read_frame_timeout(&self, timeout: Duration) -> IoResult<Self::FrameType>
    where
        Self: Sized,
    {
        use rustix::event::*;
        let pollfd = PollFd::new(self, PollFlags::IN);

        match poll(&mut [pollfd], timeout.as_millis() as c_int)? {
            0 => Err(IoErrorKind::TimedOut.into()),
            _ => {
                // let now = std::time::Instant::now();
                let res = self.read_frame();
                // dbg!(now.elapsed());
                res
            }
        }
        // let pollfd = epoll::create(epoll::CreateFlags::CLOEXEC);
        // // ::new(self, PollFlags::IN);

        // match poll(&mut [pollfd], timeout.as_millis() as c_int)? {
        //     0 => Err(IoErrorKind::TimedOut.into()),
        //     _ => {
        //         let now = std::time::Instant::now();
        //         let res = self.read_frame();
        //         dbg!(now.elapsed());
        //         res
        //     }
        // }
    }

    /// Write a single can frame.
    ///
    /// Note that this function can fail with an `EAGAIN` error or similar.
    /// Use `write_frame_insist` if you need to be sure that the message got
    /// sent or failed.
    //fn write_frame(&self, frame: &Self::FrameType) -> IoResult<()>;

    /// Writes a normal CAN 2.0 frame to the socket.
    fn write_frame<F>(&self, frame: &F) -> IoResult<()>
    where
        F: Into<Self::FrameType> + AsPtr;

    /// Blocking write a single can frame, retrying until it gets sent
    /// successfully.
    fn write_frame_insist<F>(&self, frame: &F) -> IoResult<()>
    where
        F: Into<Self::FrameType> + AsPtr,
    {
        loop {
            match self.write_frame(frame) {
                Ok(v) => return Ok(v),
                Err(e) if e.should_retry() => (),
                Err(e) => return Err(e),
            }
        }
    }
}

/// Traits for setting CAN socket options.
///
/// These are blocking calls, even when implemented on asynchronous sockets.
pub trait SocketOptions: AsRawFd {
    /// Sets an option on the socket.
    ///
    /// The libc `setsockopt` function is set to set various options on a socket.
    /// `set_socket_option` offers a somewhat type-safe wrapper that does not
    /// require messing around with `*const c_void`s.
    ///
    /// A proper `std::io::Error` will be returned on failure.
    ///
    /// Example use:
    ///
    /// ```text
    /// sock.set_socket_option(SOL_TCP, TCP_NO_DELAY, 1 as c_int)
    /// ```
    ///
    /// Note that the `val` parameter must be specified correctly; if an option
    /// expects an integer, it is advisable to pass in a `c_int`, not the default
    /// of `i32`.
    fn set_socket_option<T>(&self, level: c_int, name: c_int, val: &T) -> IoResult<()> {
        let ret = unsafe {
            libc::setsockopt(
                self.as_raw_fd(),
                level,
                name,
                val as *const _ as *const c_void,
                size_of::<T>() as socklen_t,
            )
        };

        match ret {
            0 => Ok(()),
            _ => Err(IoError::last_os_error()),
        }
    }

    /// Sets a collection of multiple socket options with one call.
    fn set_socket_option_mult<T>(&self, level: c_int, name: c_int, values: &[T]) -> IoResult<()> {
        let ret = if values.is_empty() {
            // can't pass in a ptr to a 0-len slice, pass a null ptr instead
            unsafe { libc::setsockopt(self.as_raw_fd(), level, name, ptr::null(), 0) }
        } else {
            unsafe {
                libc::setsockopt(
                    self.as_raw_fd(),
                    level,
                    name,
                    values.as_ptr().cast(),
                    size_of_val(values) as socklen_t,
                )
            }
        };

        match ret {
            0 => Ok(()),
            _ => Err(IoError::last_os_error()),
        }
    }

    /// Sets CAN ID filters on the socket.
    ///
    /// CAN packages received by SocketCAN are matched against these filters,
    /// only matching packets are returned by the interface.
    ///
    /// See `CanFilter` for details on how filtering works. By default, all
    /// single filter matching all incoming frames is installed.
    fn set_filters<F>(&self, filters: &[F]) -> IoResult<()>
    where
        F: Into<CanFilter> + Copy,
    {
        let filters: Vec<CanFilter> = filters.iter().map(|f| (*f).into()).collect();
        self.set_socket_option_mult(SOL_CAN_RAW, CAN_RAW_FILTER, &filters)
    }

    /// Disable reception of CAN frames.
    ///
    /// Sets a completely empty filter; disabling all CAN frame reception.
    fn set_filter_drop_all(&self) -> IoResult<()> {
        let filters: &[CanFilter] = &[];
        self.set_socket_option_mult(SOL_CAN_RAW, CAN_RAW_FILTER, filters)
    }

    /// Accept all frames, disabling any kind of filtering.
    ///
    /// Replace the current filter with one containing a single rule that
    /// acceps all CAN frames.
    fn set_filter_accept_all(&self) -> IoResult<()> {
        // safe unwrap: 0, 0 is a valid mask/id pair
        self.set_filters(&[(0, 0)])
    }

    /// Sets the error mask on the socket.
    ///
    /// By default (`ERR_MASK_NONE`) no error conditions are reported as
    /// special error frames by the socket. Enabling error conditions by
    /// setting `ERR_MASK_ALL` or another non-empty error mask causes the
    /// socket to receive notification about the specified conditions.
    fn set_error_filter(&self, mask: u32) -> IoResult<()> {
        self.set_socket_option(SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &mask)
    }

    /// Sets the error mask on the socket to reject all errors.
    #[inline(always)]
    fn set_error_filter_drop_all(&self) -> IoResult<()> {
        self.set_error_filter(0)
    }

    /// Sets the error mask on the socket to accept all errors.
    #[inline(always)]
    fn set_error_filter_accept_all(&self) -> IoResult<()> {
        self.set_error_filter(CAN_ERR_MASK)
    }

    /// Sets the error mask on the socket.
    ///
    /// By default (`ERR_MASK_NONE`) no error conditions are reported as
    /// special error frames by the socket. Enabling error conditions by
    /// setting `ERR_MASK_ALL` or another non-empty error mask causes the
    /// socket to receive notification about the specified conditions.
    fn set_error_mask(&self, mask: u32) -> IoResult<()> {
        self.set_socket_option(SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &mask)
    }

    /// Enable or disable loopback.
    ///
    /// By default, loopback is enabled, causing other applications that open
    /// the same CAN bus to see frames emitted by different applications on
    /// the same system.
    fn set_loopback(&self, enabled: bool) -> IoResult<()> {
        let loopback = c_int::from(enabled);
        self.set_socket_option(SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback)
    }

    /// Enable or disable receiving of own frames.
    ///
    /// When loopback is enabled, this settings controls if CAN frames sent
    /// are received back immediately by sender. Default is off.
    fn set_recv_own_msgs(&self, enabled: bool) -> IoResult<()> {
        let recv_own_msgs = c_int::from(enabled);
        self.set_socket_option(SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs)
    }

    /// Enable or disable join filters.
    ///
    /// By default a frame is accepted if it matches any of the filters set
    /// with `set_filters`. If join filters is enabled, a frame has to match
    /// _all_ filters to be accepted.
    fn set_join_filters(&self, enabled: bool) -> IoResult<()> {
        let join_filters = c_int::from(enabled);
        self.set_socket_option(SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS, &join_filters)
    }
}

// TODO: We need to restore this, but preferably with TIMESTAMPING

/*
impl CanSocket {

    /// Blocking read a single can frame with timestamp
    ///
    /// Note that reading a frame and retrieving the timestamp requires two
    /// consecutive syscalls. To avoid race conditions, exclusive access
    /// to the socket is enforce through requiring a `mut &self`.
    pub fn read_frame_with_timestamp(&mut self) -> IoResult<(CanFrame, time::SystemTime)> {
        let frame = self.read_frame()?;

        let mut ts = timespec { tv_sec: 0, tv_nsec: 0 };
        let ret = unsafe {
            libc::ioctl(self.fd, SIOCGSTAMPNS as c_ulong, &mut ts as *mut timespec)
        };

        if ret == -1 {
            return Err(IoError::last_os_error());
        }

        Ok((frame, system_time_from_timespec(ts)))
    }

}
*/

// ===== CanSocket =====

/// A socket for classic CAN 2.0 devices.
///
/// This provides an interface to read and write classic CAN 2.0 frames to
/// the bus, with up to 8 bytes of data per frame. It wraps a Linux socket
/// descriptor to a Raw SocketCAN socket.
///
/// The socket is automatically closed when the object is dropped. To close
/// manually, use std::drop::Drop. Internally this is just a wrapped socket
/// (file) descriptor.
#[allow(missing_copy_implementations)]
#[derive(Debug)]
pub struct CanSocket(socket2::Socket);

impl CanSocket {
    /// Reads a low-level libc `can_frame` from the socket.
    pub fn read_raw_frame(&self) -> IoResult<can_frame> {
        let mut frame = can_frame_uninit();
        read_uninit_exact(&self.0, as_uninit_bytes_mut(&mut frame))?;
        Ok(unsafe { frame.assume_init() })
    }
}

pub(crate) fn read_uninit_exact(fd: &impl AsFd, mut buf: &mut [MaybeUninit<u8>]) -> IoResult<()> {
    loop {
        match rustix::io::read_uninit(fd, mem::take(&mut buf)) {
            Ok((init, remaining)) => {
                if remaining.is_empty() {
                    return Ok(());
                }
                if init.is_empty() {
                    return Err(io::ErrorKind::UnexpectedEof.into());
                }
                buf = remaining;
            }
            Err(ref e) if e.kind() == io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e.into()),
        }
    }
}

impl Socket for CanSocket {
    /// CanSocket reads/writes classic CAN 2.0 frames.
    type FrameType = CanFrame;

    /// Opens the socket by interface index.
    fn open_addr(addr: &CanAddr, nonblocking: bool) -> IoResult<Self> {
        let sock = raw_open_socket(addr, nonblocking)?;
        Ok(Self(sock))
    }

    /// Writes a normal CAN 2.0 frame to the socket.
    fn write_frame<F>(&self, frame: &F) -> IoResult<()>
    where
        F: Into<CanFrame> + AsPtr,
    {
        (&self.0).write_all(frame.as_bytes())
    }

    /// Reads a normal CAN 2.0 frame from the socket.
    fn read_frame(&self) -> IoResult<CanFrame> {
        let frame = self.read_raw_frame()?;
        Ok(frame.into())
    }
}

impl SocketOptions for CanSocket {}

// Has no effect: #[deprecated(since = "3.1", note = "Use AsFd::as_fd() instead.")]
impl AsRawFd for CanSocket {
    #[inline]
    fn as_raw_fd(&self) -> RawFd {
        self.0.as_raw_fd()
    }
}

impl From<OwnedFd> for CanSocket {
    fn from(fd: OwnedFd) -> Self {
        Self(socket2::Socket::from(fd))
    }
}

impl IntoRawFd for CanSocket {
    fn into_raw_fd(self) -> RawFd {
        self.0.into_raw_fd()
    }
}

impl AsFd for CanSocket {
    #[inline]
    fn as_fd(&self) -> BorrowedFd<'_> {
        self.0.as_fd()
    }
}

impl Read for CanSocket {
    #[inline]
    fn read(&mut self, buf: &mut [u8]) -> IoResult<usize> {
        Ok(rustix::io::read(&self.0, buf)?)
    }
}

impl Write for CanSocket {
    #[inline]
    fn write(&mut self, buf: &[u8]) -> IoResult<usize> {
        Ok(rustix::io::write(&self.0, buf)?)
    }
    #[inline]
    fn flush(&mut self) -> IoResult<()> {
        Ok(())
    }
}

// ===== CanFdSocket =====

/// A socket for CAN FD devices.
///
/// This can transmit and receive CAN 2.0 frames with up to 8-bytes of data,
/// or CAN Flexible Data (FD) frames with up to 64-bytes of data.
#[allow(missing_copy_implementations)]
#[derive(Debug)]
pub struct CanFdSocket(socket2::Socket);

impl CanFdSocket {
    // Enable or disable FD mode on a socket.
    fn set_fd_mode(sock: socket2::Socket, enable: bool) -> IoResult<socket2::Socket> {
        let enable = enable as c_int;

        let ret = unsafe {
            libc::setsockopt(
                sock.as_raw_fd(),
                SOL_CAN_RAW,
                CAN_RAW_FD_FRAMES,
                &enable as *const _ as *const c_void,
                size_of::<c_int>() as u32,
            )
        };

        match ret {
            0 => Ok(sock),
            _ => Err(IoError::last_os_error()),
        }
    }

    /// Reads a raw CAN frame from the socket.
    ///
    /// This might be either type of CAN frame, a classic CAN 2.0 frame
    /// or an FD frame.
    pub fn read_raw_frame(&self) -> IoResult<CanRawFrame> {
        let mut fdframe = canfd_frame_uninit();

        match rustix::io::read_uninit(&self.0, as_uninit_bytes_mut(&mut fdframe))?
            .0
            .len()
        {
            // If we only get 'can_frame' number of bytes, then the return is,
            // by definition, a can_frame, so we just copy the bytes into the
            // proper type.
            CAN_MTU => {
                let mut frame = can_frame_uninit();
                as_uninit_bytes_mut(&mut frame)
                    .copy_from_slice(&as_uninit_bytes(&fdframe)[..CAN_MTU]);
                Ok(unsafe { frame.assume_init().into() })
            }
            CANFD_MTU => Ok(unsafe { fdframe.assume_init().into() }),
            _ => Err(IoError::last_os_error()),
        }
    }
}

impl Socket for CanFdSocket {
    /// CanFdSocket can read/write classic CAN 2.0 or FD frames.
    type FrameType = CanAnyFrame;

    /// Opens the FD socket by interface index.
    fn open_addr(addr: &CanAddr, nonblocking: bool) -> IoResult<Self> {
        raw_open_socket(addr, nonblocking)
            .and_then(|sock| Self::set_fd_mode(sock, true))
            .map(Self)
    }

    /// Writes any type of CAN frame to the socket.
    fn write_frame<F>(&self, frame: &F) -> IoResult<()>
    where
        F: Into<Self::FrameType> + AsPtr,
    {
        (&self.0).write_all(frame.as_bytes())
    }

    /// Reads either type of CAN frame from the socket.
    fn read_frame(&self) -> IoResult<CanAnyFrame> {
        let mut fdframe = canfd_frame_uninit();

        match rustix::io::read_uninit(&self.0, as_uninit_bytes_mut(&mut fdframe))?
            .0
            .len()
        {
            // If we only get 'can_frame' number of bytes, then the return is,
            // by definition, a can_frame, so we just copy the bytes into the
            // proper type.
            CAN_MTU => {
                let mut frame = can_frame_uninit();
                as_uninit_bytes_mut(&mut frame)
                    .copy_from_slice(&as_uninit_bytes(&fdframe)[..CAN_MTU]);
                Ok(CanFrame::from(unsafe { frame.assume_init() }).into())
            }
            CANFD_MTU => Ok(CanFdFrame::from(unsafe { fdframe.assume_init() }).into()),
            _ => Err(IoError::last_os_error()),
        }
    }
}

impl SocketOptions for CanFdSocket {}

// Has no effect: #[deprecated(since = "3.1", note = "Use AsFd::as_fd() instead.")]
impl AsRawFd for CanFdSocket {
    fn as_raw_fd(&self) -> RawFd {
        self.0.as_raw_fd()
    }
}

impl From<OwnedFd> for CanFdSocket {
    fn from(fd: OwnedFd) -> CanFdSocket {
        Self(socket2::Socket::from(fd))
    }
}

impl IntoRawFd for CanFdSocket {
    fn into_raw_fd(self) -> RawFd {
        self.0.into_raw_fd()
    }
}

impl AsFd for CanFdSocket {
    fn as_fd(&self) -> BorrowedFd<'_> {
        self.0.as_fd()
    }
}

impl Read for CanFdSocket {
    fn read(&mut self, buf: &mut [u8]) -> IoResult<usize> {
        self.0.read(buf)
    }
}

impl Write for CanFdSocket {
    fn write(&mut self, buf: &[u8]) -> IoResult<usize> {
        self.0.write(buf)
    }

    fn flush(&mut self) -> IoResult<()> {
        self.0.flush()
    }
}

// ===== CanFilter =====

/// The CAN filter defines which ID's can be accepted on a socket.
///
/// Each filter contains an internal id and mask. Packets are considered to
/// be matched by a filter if `received_id & mask == filter_id & mask` holds
/// true.
///
/// A socket can be given multiple filters, and each one can be inverted
/// ([ref](https://docs.kernel.org/networking/can.html#raw-protocol-sockets-with-can-filters-sock-raw))
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct CanFilter(libc::can_filter);

impl CanFilter {
    /// Construct a new CAN filter.
    pub fn new(id: canid_t, mask: canid_t) -> Self {
        Self(libc::can_filter {
            can_id: id,
            can_mask: mask,
        })
    }

    /// Construct a new inverted CAN filter.
    pub fn new_inverted(id: canid_t, mask: canid_t) -> Self {
        Self::new(id | libc::CAN_INV_FILTER, mask)
    }
}

impl From<libc::can_filter> for CanFilter {
    fn from(filt: libc::can_filter) -> Self {
        Self(filt)
    }
}

impl From<(u32, u32)> for CanFilter {
    fn from(filt: (u32, u32)) -> Self {
        CanFilter::new(filt.0, filt.1)
    }
}

impl AsRef<libc::can_filter> for CanFilter {
    fn as_ref(&self) -> &libc::can_filter {
        &self.0
    }
}
