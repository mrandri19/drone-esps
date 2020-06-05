use ring_channel::*;
use std::num::NonZeroUsize;

use serialport::{open_with_settings, SerialPortSettings};
use std::io;
use std::net::TcpListener;
use std::thread;
use std::thread::spawn;
use tungstenite::server::accept;
fn main() {
    let (mut producer, consumer) = ring_channel::<Vec<f32>>(NonZeroUsize::new(100).unwrap());

    let freq = 200;
    let period = std::time::Duration::from_millis(1000 / freq);
    let settings = SerialPortSettings {
        baud_rate: 115200,
        timeout: period,
        ..Default::default()
    };
    let mut s = open_with_settings("/dev/ttyUSB0", &settings).unwrap();
    s.clear(serialport::ClearBuffer::All).unwrap();

    let mut serial_buf: Vec<u8> = vec![0; 1000];
    let mut str_buf = String::new();

    let serial_thread = spawn(move || loop {
        match s.read(serial_buf.as_mut_slice()) {
            Ok(t) => {
                // TODO(Andrea): Shitty parsing code
                str_buf.push_str(std::str::from_utf8(&serial_buf[..t]).unwrap());

                if str_buf.ends_with('\n') {
                    str_buf = str_buf.trim().to_string();
                    println!("{}", str_buf);
                    let data = str_buf
                        .split(", ")
                        .flat_map(|s| s.split(" "))
                        .map(|s| s.parse())
                        .flat_map(|s| s)
                        .collect::<Vec<f32>>();

                    producer.send(data).unwrap();

                    str_buf = "".to_string();
                }
            }
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
            Err(e) => eprintln!("{:?}", e),
        }
    });

    let server = TcpListener::bind("127.0.0.1:8000").unwrap();

    // For each connection, like an infinite loop + accept
    for stream in server.incoming() {
        // Create a clone for the RWLock, which is then moved by the thread
        let mut consumer_clone = consumer.clone();
        // Spawn a new thread taking ownership of that connection's stream
        spawn(move || {
            // Accept the given stream as a websocket.
            // Start a server ws handshake over the stream
            let mut websocket = accept(stream.unwrap()).unwrap();

            loop {
                let r = consumer_clone.recv().unwrap();
                let msg = tungstenite::Message::Text(format!("{:?}", r));

                websocket.write_message(msg).unwrap();
            }
        });
    }

    serial_thread.join().unwrap();
}
