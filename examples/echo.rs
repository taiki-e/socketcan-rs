// socketcan/examples/echo.rs
//
// This file is part of the Rust 'socketcan-rs' library.
//
// Licensed under the MIT license:
//   <LICENSE or http://opensource.org/licenses/MIT>
// This file may not be copied, modified, or distributed except according
// to those terms.
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Jul 05 2022
//

use anyhow::Context;
use embedded_can::{blocking::Can, Frame as EmbeddedFrame, StandardId};
use socketcan::{CanFdFrame, CanFdSocket, CanFrame, CanSocket, Frame, Socket};
use std::{
    env,
    io::Write,
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};

fn frame_to_string<F: Frame>(frame: &F) -> String {
    let id = frame.raw_id();

    let data_string = frame
        .data()
        .iter()
        .fold(String::from(""), |a, b| format!("{} {:02x}", a, b));

    format!("{:08X}  [{}] {}", id, frame.dlc(), data_string)
}
fn print_frame<F: Frame>(frame: &F) {
    let id = frame.raw_id();

    let mut stdout = std::io::stdout().lock();
    let _ = write!(stdout, "{:08X}  [{}] ", id, frame.dlc());
    frame.data().iter().for_each(|a| {
        let _ = write!(stdout, " {:02x}", a);
    });
    let _ = writeln!(stdout);
    let _ = stdout.flush();
}

fn main() -> anyhow::Result<()> {
    let iface = env::args().nth(1).unwrap_or_else(|| "vcan0".into());

    let sock = CanSocket::open(&iface)
        .with_context(|| format!("Failed to open socket on interface {}", iface))?;
    // let sock = CanFdSocket::open(&iface)
    //     .with_context(|| format!("Failed to open socket on interface {}", iface))?;

    static QUIT: AtomicBool = AtomicBool::new(false);
    static READY: AtomicBool = AtomicBool::new(false);

    ctrlc::set_handler(|| {
        QUIT.store(true, Ordering::Relaxed);
        std::process::exit(1)
    })
    .expect("Failed to set ^C handler");

    let h = std::thread::spawn(move || {
        READY.store(true, Ordering::Release);
        while !QUIT.load(Ordering::Relaxed) {
            if sock.read_frame_timeout(Duration::from_millis(100)).is_ok() {
                let now = Instant::now();
                for _ in 0..1000 {
                    let frame = sock.read_frame().unwrap();
                    // std::hint::black_box(&frame);
                    print_frame(&frame);
                    // println!("{}", frame_to_string(&frame));

                    // let new_id = frame.raw_id() + 0x01;
                    // let new_id = StandardId::new(new_id as u16).expect("Failed to create ID");

                    // let echo_frame = CanFrame::new(new_id, frame.data()).unwrap();
                    // sock.transmit(&echo_frame)
                    //     .expect("Failed to echo received frame");
                }
                dbg!(now.elapsed());
            }
        }
    });

    while !READY.load(Ordering::Acquire) {}
    let socket_tx = CanSocket::open(&iface).unwrap();
    // let socket_tx = CanFdSocket::open(&iface).unwrap();

    let id = StandardId::new(0x100).unwrap();
    let frame = CanFrame::new(id, &[0, 0, 0, 0, 0, 0, 0, 0]).unwrap();
    // let frame = CanFdFrame::new(id, &[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();

    // println!("Writing on {}", iface);
    while !QUIT.load(Ordering::Relaxed) {
        socket_tx.write_frame(&frame).unwrap();
    }
    h.join().unwrap();
    Ok(())
}
