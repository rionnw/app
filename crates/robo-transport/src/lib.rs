use anyhow::{Context, Result};
use robo_core::{Steps, Transport};
use serialport::SerialPort;
use std::io::{Read, Write};
use std::time::Duration;

pub const FRAME_HEAD: u8 = 0xAA;
pub const FRAME_TAIL: u8 = 0xBB;
pub const FRAME_SIZE: usize = 150;

pub struct SerialTransport {
    port: Box<dyn SerialPort>,
}

impl SerialTransport {
    pub fn open(port_name: &str, baud_rate: u32) -> Result<Self> {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(100))
            .open()
            .with_context(|| format!("failed to open serial port {port_name}"))?;
        Ok(Self { port })
    }

    pub fn read_available(&mut self) -> Result<Vec<u8>> {
        let mut buf = vec![0u8; 256];
        match self.port.read(&mut buf) {
            Ok(n) => {
                buf.truncate(n);
                Ok(buf)
            }
            Err(err) if err.kind() == std::io::ErrorKind::TimedOut => Ok(Vec::new()),
            Err(err) => Err(err).context("failed to read serial port"),
        }
    }
}

impl Transport for SerialTransport {
    fn send_steps(&mut self, steps: &Steps) -> Result<()> {
        let frame = encode_motion_frame(steps.encoded.as_bytes())?;
        self.port
            .write_all(&frame)
            .context("failed to write serial frame")?;
        self.port.flush().context("failed to flush serial port")?;
        Ok(())
    }
}

pub fn encode_motion_frame(payload: &[u8]) -> Result<[u8; FRAME_SIZE]> {
    anyhow::ensure!(
        payload.len() <= FRAME_SIZE - 2,
        "payload too long: {} bytes",
        payload.len()
    );
    let mut frame = [b'Z'; FRAME_SIZE];
    frame[0] = FRAME_HEAD;
    frame[1..1 + payload.len()].copy_from_slice(payload);
    frame[FRAME_SIZE - 1] = FRAME_TAIL;
    Ok(frame)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_fixed_size_frame() {
        let frame = encode_motion_frame(b"ABC").unwrap();
        assert_eq!(frame[0], FRAME_HEAD);
        assert_eq!(&frame[1..4], b"ABC");
        assert_eq!(frame[149], FRAME_TAIL);
    }
}
