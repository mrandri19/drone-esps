use serialport::{open_with_settings, SerialPortSettings};
use std::io::{self, Write};
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
fn main() {
    let freq = 200;
    let period = std::time::Duration::from_millis(1000 / freq);
    let settings = SerialPortSettings {
        baud_rate: 961200,
        timeout: period,
        ..Default::default()
    };
    let mut s = open_with_settings("/dev/ttyUSB0", &settings).unwrap();
    // let mut clone = s.try_clone().unwrap();

    // let ready = Arc::new(AtomicBool::new(false));

    // std::thread::spawn(move || {
    let mut i = 0;
    // while !read_clone.load(Ordering::SeqCst) {}
    loop {
        s.write(format!("{}", i).as_bytes()).unwrap();
        // clone.flush().unwrap();

        std::thread::sleep(period);
        i += 1;
    }
    // });

    // let mut serial_buf: Vec<u8> = vec![0; 1000];
    // let mut str_buf = String::new();
    // loop {
    //     match s.read(serial_buf.as_mut_slice()) {
    //         Ok(t) => {
    //             str_buf.push_str(std::str::from_utf8(&serial_buf[..t]).unwrap());
    //             if str_buf.ends_with('\n') {
    //                 print!("{}", str_buf);
    //                 if str_buf.contains("DHCP server assigned IP to a station") {
    //                     ready.store(true, Ordering::SeqCst);
    //                 }
    //                 str_buf = "".to_string();
    //             }
    //         }
    //         Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
    //         Err(e) => eprintln!("{:?}", e),
    //     }
    // }
}
