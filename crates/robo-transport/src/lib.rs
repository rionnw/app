use anyhow::{Context, Result};
use robo_core::{DigitMap, Transport, MNEMONIC_COUNT};
use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{Read, Write};
use std::time::Duration;

pub const FRAME_HEAD: u8 = 0xAA;
pub const FRAME_TAIL: u8 = 0xBB;
pub const FRAME_SIZE: usize = 150;

/// mnemonic 助记符表（与 robo-handstep `MNEMONIC_STR` / robo-core `MNEMONIC_COUNT` 顺序一致）
pub const MNEMONICS: [&str; MNEMONIC_COUNT] = [
    "M_L1", "M_L2", "M_L3", "M_LC", "M_LO",
    "M_R1", "M_R2", "M_R3", "M_RC", "M_RO",
];

/// mnemonic 槽位数量；从 robo-core 转出便于上层直接 use。
pub const MOVE_COUNT: usize = MNEMONIC_COUNT;

/// 默认数字映射（与硬件出厂约定一致，用户未自定义时使用）
pub const DEFAULT_DIGIT_MAP: [&str; MOVE_COUNT] = [
    "4", "3", "2", "0", "1",
    "9", "8", "7", "5", "6",
];

/// 构造默认 `DigitMap`。
pub fn default_digit_map() -> DigitMap {
    let mut out: DigitMap = Default::default();
    for (i, s) in DEFAULT_DIGIT_MAP.iter().enumerate() {
        out[i] = (*s).to_string();
    }
    out
}

pub struct SerialTransport {
    port: Box<dyn SerialPort>,
}

impl SerialTransport {
    /// 打开串口；参数与桌面端 RobotApp（Qt QSerialPort）保持一致：
    /// 8 数据位、无校验、1 停止位、无流控；读超时 100ms。
    pub fn open(port_name: &str, baud_rate: u32) -> Result<Self> {
        let port = serialport::new(port_name, baud_rate)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
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

    /// 发送已编码的下位机数字串（不再做 mnemonic→digit 转换）。
    /// 调用方自己用 `encode_mnemonics` 把 mnemonic 列表转成数字串。
    pub fn send_encoded(&mut self, encoded: &str) -> Result<()> {
        let frame = encode_motion_frame(encoded.as_bytes())?;
        // 与桌面端 sendMsg 第 227 行 serialPort->clear() 对齐：
        // 发送前清空收发缓冲，避免上一次未读完的回包干扰本帧。
        if let Err(err) = self.port.clear(ClearBuffer::All) {
            log::warn!("clear serial buffer before send failed: {err}");
        }
        self.port
            .write_all(&frame)
            .context("failed to write serial frame")?;
        self.port.flush().context("failed to flush serial port")?;
        Ok(())
    }
}

impl Transport for SerialTransport {
    fn send_steps(&mut self, mnemonics: &[String], digit_map: &DigitMap) -> Result<()> {
        let encoded = encode_mnemonics(mnemonics, digit_map);
        self.send_encoded(&encoded)
    }
}

/// 把 mnemonic 序列按 digit_map 编码成下位机字符串。
///
/// `digit_map` 索引顺序与 robo-handstep 的 `MNEMONIC_STR` 一致：
/// `[M_L1, M_L2, M_L3, M_LC, M_LO, M_R1, M_R2, M_R3, M_RC, M_RO]`。
/// 不在该集合的 mnemonic 会被跳过并打 warn 日志（不应正常发生）。
pub fn encode_mnemonics(mnemonics: &[String], digit_map: &DigitMap) -> String {
    let mut out = String::with_capacity(mnemonics.len() * 2);
    for m in mnemonics {
        match MNEMONICS.iter().position(|x| *x == m.as_str()) {
            Some(idx) => out.push_str(&digit_map[idx]),
            None => log::warn!("unknown mnemonic {:?} (skipped)", m),
        }
    }
    out
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
