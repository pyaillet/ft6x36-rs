#![no_std]
#![doc = include_str!("../README.md")]

use embedded_hal::blocking::i2c::{Write, WriteRead};

use num_enum::{FromPrimitive, IntoPrimitive};

const DEFAULT_FT6X36_ADDRESS: u8 = 0x38;
const REPORT_SIZE: usize = 0x0f;

/// Driver representation holding:
///
/// - The I2C Slave address of the device
/// - The I2C Bus used to communicate with the device
/// - Some information on the device when it's initialized
pub struct Ft6x36<I2C> {
    /// Address of the I2C Slave device
    address: u8,
    /// I2C bus used to communicate with the device
    i2c: I2C,
    /// Information of the device when it's initialized
    info: Option<Ft6x36Info>,
}

#[derive(Copy, Clone, Debug)]
pub struct Diagnostics {
    power_mode: u8,
    g_mode: u8,
    lib_version: u16,
    state: u8,
    control_mode: u8
}

#[derive(Copy, Clone, Debug)]
pub struct Point {
    pub x: u16,
    pub y: u16,
}

impl From<&[u8]> for Point {
    fn from(data: &[u8]) -> Self {
        let x: u16 = (data[0] as u16 & 0x0f << 8) | (data[1] as u16);
        let y: u16 = (data[2] as u16 & 0x0f << 8) | (data[3] as u16);
        Self { x, y }
    }
}

pub enum Direction {
    Up,
    Down,
    Left,
    Right,
}

pub enum Zoom {
    ZoomIn(Point),
    ZoomOut(Point),
}

pub struct SwipeInfo {
    pub velocity: u8,
    pub point: Point,
}

pub enum TouchEvent {
    NoEvent,
    TouchOnePoint(Point),
    TouchTwoPoint(Point, Point),
    Swipe(Direction, SwipeInfo),
    Zoom(Zoom),
}

/// Device mode
#[repr(u8)]
#[derive(Clone, Copy, Debug, FromPrimitive, IntoPrimitive)]
pub enum DeviceMode {
    /// Working mode
    #[default]
    Working = 0b000,
    /// Factory mode
    Factory = 0b100,
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, IntoPrimitive)]
pub enum TouchType {
    Press = 0b00,
    Release = 0b01,
    Contact = 0b10,
    Invalid = 0b11,
}

impl From<u8> for TouchType {
    fn from(data: u8) -> Self {
        match (data >> 6) & 0b0000_0011 {
            0b00 => TouchType::Press,
            0b01 => TouchType::Release,
            0b10 => TouchType::Contact,
            _ => TouchType::Invalid,
        }
    }
}

/// Touch event full raw report
#[allow(dead_code)]
#[derive(Debug)]
pub struct RawTouchEvent {
    /// Device mode
    device_mode: DeviceMode,
    gesture_id: GestureId,
    touch_type: TouchType,
    p1: Option<Point>,
    p2: Option<Point>,
}

/// Settings for gesture detection
/// (Currently not working)
#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub struct GestureParams {
    minimum_angle: u8,
    offset_left_right: u8,
    offset_up_down: u8,
    dist_left_right: u8,
    dist_up_down: u8,
    dist_zoom: u8,
}

macro_rules! get_offset {
    ($first:expr, $relative_offset:expr) => {
        ($relative_offset as u8 - $first as u8) as usize
    };
}

/// Documented registers of the device
#[allow(dead_code)]
#[repr(u8)]
#[derive(Debug, Clone, Copy, IntoPrimitive)]
enum Reg {
    DeviceMode = 0x00,

    GestId = 0x01,
    TouchDeviceStatus = 0x02,

    P1XH = 0x03,
    P1XL = 0x04,
    P1YH = 0x05,
    P1YL = 0x06,
    P1Weight = 0x07,
    P1Misc = 0x08,

    P2XH = 0x09,
    P2XL = 0x0A,
    P2YH = 0x0B,
    P2YL = 0x0C,
    P2Weight = 0x0D,
    P2Misc = 0x0E,

    TouchDetectionThreshold = 0x80,
    TouchFilterCoeff = 0x85,
    ControlMode = 0x86,
    TimeActiveMonitor = 0x87,
    PeriodActive = 0x88,
    PeriodMonitor = 0x89,

    GestRadianValue = 0x91,
    GestOffsetLeftRight = 0x92,
    GestOffsetUpDown = 0x93,
    GestDistLeftRight = 0x94,
    GestDistUpDown = 0x95,
    GestDistZoom = 0x96,

    LibVersionH = 0xA1,
    LibVersionL = 0xA2,
    ChipId = 0xA3,

    GMode = 0xA4,
    PowerMode = 0xA5,
    FirmwareId = 0xA6,
    PanelId = 0xA8,
    ReleaseCode = 0xAF,

    OperatingMode = 0xBC,
}

/// Known and detected gestures (currently not working though)
#[repr(u8)]
#[derive(Clone, Copy, Debug, IntoPrimitive, FromPrimitive)]
pub enum GestureId {
    #[default]
    NoGesture = 0x00,
    MoveUp = 0x10,
    MoveRight = 0x14,
    MoveDown = 0x18,
    MoveLeft = 0x1C,
    ZoomIn = 0x48,
    ZoomOut = 0x49,
}

/// Enum describing known chips
#[repr(u8)]
#[derive(Clone, Copy, Debug, IntoPrimitive, FromPrimitive)]
pub enum ChipId {
    #[default]
    Unknown,
    Ft6206 = 0x06,
    Ft6236 = 0x36,
    Ft6236u = 0x64,
}

/// A structure giving information on the current device
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub struct Ft6x36Info {
    /// ChipId, known chips are: Ft6206, Ft6236 and Ft6236u
    chip_id: ChipId,
    /// Firmware Id
    firmware_id: u8,
    /// Panel id
    panel_id: u8,
    /// Version of the release code
    release_code: u8,
}

impl<I2C, E> Ft6x36<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    /// Create a new Ft6x36 device with the default slave address
    ///
    /// # Arguments
    ///
    /// - `i2c` I2C bus used to communicate with the device
    ///
    /// # Returns
    ///
    /// - [Ft6x36 driver](Ft6x36) created
    ///
    pub fn new(i2c: I2C) -> Self {
        Self {
            address: DEFAULT_FT6X36_ADDRESS,
            i2c,
            info: None,
        }
    }

    /// Initialize the device
    ///
    /// Currently it only gather informations on the device and initialize the
    /// [info structure of the driver](Ft6x36Info)
    ///
    pub fn init(&mut self) -> Result<(), E> {
        let mut buf: [u8; 13] = [0; 13];
        self.i2c
            .write_read(self.address, &[Reg::ChipId.into()], &mut buf)?;
        let chip_id: ChipId = buf[get_offset!(Reg::ChipId, Reg::ChipId)].into();
        let firmware_id: u8 = buf[get_offset!(Reg::ChipId, Reg::FirmwareId)];
        let panel_id: u8 = buf[get_offset!(Reg::ChipId, Reg::PanelId)];
        let release_code: u8 = buf[get_offset!(Reg::ChipId, Reg::ReleaseCode)];
        self.info = Some(Ft6x36Info {
            chip_id,
            firmware_id,
            panel_id,
            release_code,
        });
        self.set_control_mode(0)?;
        Ok(())
    }

    /// Get the full raw report of touch events
    ///
    /// # Returns
    ///
    /// - [TouchEvent](TouchEvent) the full TouchEvent report
    pub fn get_touch_event(&mut self) -> Result<RawTouchEvent, E> {
        let mut report: [u8; REPORT_SIZE] = [0; REPORT_SIZE];
        self.i2c
            .write_read(self.address, &[Reg::DeviceMode.into()], &mut report)?;

        let touch_type: TouchType = report[3].into();

        let (p1, p2) = match report[2] {
            1 => (Some(Point::from(&report[3..7])), None),
            2 => (
                Some(Point::from(&report[3..7])),
                Some(Point::from(&report[9..13])),
            ),
            _ => (None, None),
        };

        Ok(RawTouchEvent {
            device_mode: report[0].into(),
            gesture_id: report[1].into(),
            touch_type,
            p1,
            p2,
        })
    }

    /// Get the current gesture detection parameters
    /// (Currently not working)
    pub fn get_gesture_params(&mut self) -> Result<GestureParams, E> {
        let mut buf: [u8; 6] = [0; 6];

        self.i2c
            .write_read(self.address, &[Reg::GestRadianValue.into()], &mut buf)?;
        Ok(GestureParams {
            minimum_angle: buf[0],
            offset_left_right: buf[1],
            offset_up_down: buf[2],
            dist_left_right: buf[3],
            dist_up_down: buf[4],
            dist_zoom: buf[5],
        })
    }

    /// Set the touch detection threshold
    ///
    /// # Arguments
    ///
    /// - `value` the threshold value
    pub fn set_touch_threshold(&mut self, value: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Reg::TouchDetectionThreshold.into(), value])
    }

    /// Set the touch filter coefficient
    ///
    /// # Arguments
    ///
    /// - `value` the touch filter coefficient
    pub fn set_touch_filter_coefficient(&mut self, value: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Reg::TouchFilterCoeff.into(), value])
    }

    /// Set the control mode
    ///
    /// - 0: Will keep the Active mode when there is no touching
    /// - 1: Switching from Active mode to Monitor mode automatically when there
    ///   is no touching and the TimeActiveMonitor period is elapsed
    ///
    /// # Arguments
    ///
    /// - `value` the control mode
    pub fn set_control_mode(&mut self, value: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Reg::ControlMode.into(), value])
    }

    /// Set the period used to switch from Active to Monitor mode
    ///
    /// # Arguments
    ///
    /// - `value` the switching period
    pub fn set_time_active_monitor(&mut self, value: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Reg::TimeActiveMonitor.into(), value])
    }

    /// Set the report rate in Active mode
    ///
    /// # Arguments
    ///
    /// - `value` the report rate in Active mode
    pub fn set_period_active(&mut self, value: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Reg::PeriodActive.into(), value])
    }

    /// Set the report rate in Monitor mode
    ///
    /// # Arguments
    ///
    /// - `value` the report rate in Monitor mode
    pub fn set_period_monitor(&mut self, value: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Reg::PeriodMonitor.into(), value])
    }

    /// Set the minimum angle for gesture detection
    ///
    /// # Arguments
    ///
    /// - `value` The value of the minimum allowed angle while rotating gesture
    ///   mode
    pub fn set_gesture_minimum_angle(&mut self, value: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Reg::GestRadianValue.into(), value])
    }

    /// Set the maximum offset for detecting Moving left and Moving right gestures
    ///
    /// # Arguments
    ///
    /// - `value` The value of the maximum offset for detecting Moving left and Moving right gestures
    ///   mode
    pub fn set_gesture_offset_left_right(&mut self, value: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Reg::GestOffsetLeftRight.into(), value])
    }

    /// Set the maximum offset for detecting Moving up and Moving down gestures
    ///
    /// # Arguments
    ///
    /// - `value` The value of the maximum offset for detecting Moving up and Moving down gestures
    ///   mode
    pub fn set_gesture_offset_up_down(&mut self, value: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Reg::GestOffsetUpDown.into(), value])
    }

    /// Set the minimum distance for detecting Moving up and Moving down gestures
    ///
    /// # Arguments
    ///
    /// - `value` The value of the minimum distance for detecting Moving up and Moving down gestures
    ///   mode
    pub fn set_gesture_distance_up_down(&mut self, value: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Reg::GestDistUpDown.into(), value])
    }

    /// Set the minimum distance for detecting Moving left and Moving right gestures
    ///
    /// # Arguments
    ///
    /// - `value` The value of the minimum distance for detecting Moving left and Moving right
    ///   gestures mode
    pub fn set_gesture_distance_left_right(&mut self, value: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Reg::GestDistLeftRight.into(), value])
    }

    /// Set the minimum distance for detecting zoom gestures
    ///
    /// # Arguments
    ///
    /// - `value` The value of the minimum distance for detecting zoom gestures mode
    pub fn set_gesture_distance_zoom(&mut self, value: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Reg::GestDistZoom.into(), value])
    }

    /// Get device informations
    ///
    /// # Returns
    ///
    /// - `None` if the device is not initialized
    /// - [`Some(Ft6x36Info)`](Ft6x36Info) otherwise
    pub fn get_info(&self) -> Option<Ft6x36Info> {
        self.info
    }

    /// Get device Diagnostics
    ///
    /// # Returns
    ///
    /// - 
    pub fn get_diagnostics(&mut self) -> Result<Diagnostics, E> {
        let mut buf: [u8; 1] = [0];
        self.i2c.write_read(self.address, &[Reg::PowerMode.into()], &mut buf)?;
        let power_mode = buf[0];
        self.i2c.write_read(self.address, &[Reg::GMode.into()], &mut buf)?;
        let g_mode = buf[0];
        self.i2c.write_read(self.address, &[Reg::OperatingMode.into()], &mut buf)?;
        let state = buf[0];
        self.i2c.write_read(self.address, &[Reg::LibVersionH.into()], &mut buf)?;
        let lib_version_h = buf[0];
        self.i2c.write_read(self.address, &[Reg::LibVersionL.into()], &mut buf)?;
        let lib_version_l = buf[0];
        let lib_version: u16 = (lib_version_h as u16) << 8 | lib_version_l as u16;
        self.i2c.write_read(self.address, &[Reg::ControlMode.into()], &mut buf)?;
        let control_mode = buf[0];
        Ok(Diagnostics {
            power_mode,
            g_mode,
            state,
            lib_version,
            control_mode,
        })
    }
}
