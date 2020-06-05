use serialport::{open_with_settings, SerialPortSettings};
use std::io;
use std::net::TcpListener;
use std::sync::{Arc, RwLock};
use std::thread;
use std::thread::spawn;
use tungstenite::server::accept;
fn main() {
    // TODO(Andrea): Instead of having a single item how about deque
    let lock = Arc::new(RwLock::new(vec![0f32, 0f32]));
    let c_lock = lock.clone();

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
                    let err_roll: Vec<f32> = str_buf
                        .split(" ")
                        .map(|s| s.trim().to_string().parse::<f32>())
                        .flat_map(|r| r)
                        .collect();
                    if err_roll.len() == 2 {
                        let mut w = lock.write().unwrap();
                        *w = err_roll.clone();
                    }

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
        let cc_lock = c_lock.clone();
        // Spawn a new thread taking ownership of that connection's stream
        spawn(move || {
            // Accept the given stream as a websocket.
            // Start a server ws handshake over the stream
            let mut websocket = accept(stream.unwrap()).unwrap();

            loop {
                let r = cc_lock.read().unwrap();
                let msg = tungstenite::Message::Text(format!("{:?}", *r));

                websocket.write_message(msg).unwrap();

                thread::sleep(std::time::Duration::from_millis(15));
            }
        });
    }

    serial_thread.join().unwrap();
}
