#![no_std]
#![doc = include_str!("../README.md")]

use embedded_hal::i2c::{ErrorType, I2c, SevenBitAddress};
use num_enum::{FromPrimitive, IntoPrimitive};

#[cfg(feature = "event_process")]
use core::time::Duration;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

const DEFAULT_FT6X36_ADDRESS: u8 = 0x38;
const REPORT_SIZE: usize = 0x0f;

#[cfg(feature = "event_process")]
const MAX_DELTA_TOUCH_EVENT: Duration = Duration::from_millis(200);

/// Represents the dimensions of the device
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Dimension(pub u16, pub u16);

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
    #[cfg(feature = "event_process")]
    /// Raw events
    events: (Option<TimedRawTouchEvent>, Option<TimedRawTouchEvent>),
    #[cfg(feature = "event_process")]
    /// Event process config
    config: ProcessEventConfig,
    /// Orientation of the screen
    orientation: Orientation,
    /// Dimensions of the device
    size: Dimension,
}

/// Represents the orientation of the device
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Orientation {
    Portrait,
    Landscape,
    InvertedPortrait,
    InvertedLandscape,
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg(feature = "event_process")]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ProcessEventConfig {
    gesture_timing: Duration,
    max_swipe_delta: u16,
    min_swipe_delta: u16,
}

#[cfg(feature = "event_process")]
impl Default for ProcessEventConfig {
    fn default() -> Self {
        ProcessEventConfig {
            gesture_timing: Duration::from_millis(800),
            max_swipe_delta: 30,
            min_swipe_delta: 30,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg(feature = "event_process")]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq)]
pub struct TimedRawTouchEvent {
    time: Duration,
    event: RawTouchEvent,
}

#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Diagnostics {
    power_mode: u8,
    g_mode: u8,
    lib_version: u16,
    state: u8,
    control_mode: u8,
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TouchPoint {
    pub touch_type: TouchType,
    pub x: u16,
    pub y: u16,
}

impl TouchPoint {
    fn translate_coordinates(self, size: Dimension, orientation: Orientation) -> Self {
        fn difference_or_zero(o1: u16, o2: u16) -> u16 {
            if o2 > o1 {
                0
            } else {
                o1 - o2
            }
        }
        let TouchPoint { touch_type, x, y } = self;
        let (x, y) = match orientation {
            Orientation::Portrait => (x, y),
            Orientation::InvertedPortrait => {
                (difference_or_zero(size.0, x), difference_or_zero(size.1, y))
            }
            Orientation::Landscape => (y, difference_or_zero(size.0, x)),
            Orientation::InvertedLandscape => (difference_or_zero(size.1, y), x),
        };
        TouchPoint { x, y, touch_type }
    }
}

impl From<&[u8]> for TouchPoint {
    fn from(data: &[u8]) -> Self {
        let x: u16 = ((data[0] as u16 & 0x0f) << 8) | (data[1] as u16);
        let y: u16 = ((data[2] as u16 & 0x0f) << 8) | (data[3] as u16);
        let touch_type: TouchType = data[0].into();

        Self { touch_type, x, y }
    }
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Direction {
    Up,
    Down,
    Left,
    Right,
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Zoom {
    ZoomIn(TouchPoint),
    ZoomOut(TouchPoint),
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SwipeInfo {
    pub velocity: u16,
    pub point: TouchPoint,
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TouchEvent {
    TouchOnePoint(TouchPoint),
    TouchTwoPoint(TouchPoint, TouchPoint),
    Swipe(Direction, SwipeInfo),
    Zoom(Zoom),
}

/// Device mode
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(u8)]
#[derive(Clone, Copy, Debug, FromPrimitive, IntoPrimitive, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceMode {
    /// Working mode
    #[default]
    Working = 0b000,
    /// Factory mode
    Factory = 0b100,
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(u8)]
#[derive(Clone, Copy, Debug, IntoPrimitive, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[allow(dead_code)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawTouchEvent {
    /// Device mode
    pub device_mode: DeviceMode,
    pub gesture_id: GestureId,
    pub p1: Option<TouchPoint>,
    pub p2: Option<TouchPoint>,
}

impl From<[u8; REPORT_SIZE]> for RawTouchEvent {
    fn from(report: [u8; REPORT_SIZE]) -> Self {
        let (p1, p2) = match report[2] {
            1 => (Some(TouchPoint::from(&report[3..7])), None),
            2 => (
                Some(TouchPoint::from(&report[3..7])),
                Some(TouchPoint::from(&report[9..13])),
            ),
            _ => (None, None),
        };

        RawTouchEvent {
            device_mode: report[0].into(),
            gesture_id: report[1].into(),
            p1,
            p2,
        }
    }
}

impl RawTouchEvent {
    fn translate_orientation(self, size: Dimension, orientation: Orientation) -> Self {
        let RawTouchEvent {
            device_mode,
            gesture_id,
            p1,
            p2,
        } = self;
        let p1 = p1.map(|p| p.translate_coordinates(size, orientation));
        let p2 = p2.map(|p| p.translate_coordinates(size, orientation));
        RawTouchEvent {
            device_mode,
            gesture_id,
            p1,
            p2,
        }
    }
}

/// Settings for gesture detection
/// (Currently not working)
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(u8)]
#[derive(Clone, Copy, Debug, IntoPrimitive, FromPrimitive, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(u8)]
#[derive(Clone, Copy, Debug, IntoPrimitive, FromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChipId {
    #[default]
    Unknown,
    Ft6206 = 0x06,
    Ft6236 = 0x36,
    Ft6236u = 0x64,
}

/// A structure giving information on the current device
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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

impl<I2C> Ft6x36<I2C>
where
    I2C: I2c<SevenBitAddress>,
{
    /// Create a new Ft6x36 device with the default slave address
    ///
    /// # Arguments
    ///
    /// - `i2c` I2C bus used to communicate with the device
    /// - `size` [`Dimension`] of the device
    ///
    /// # Returns
    ///
    /// - [Ft6x36 driver](Ft6x36) created
    ///
    pub fn new(i2c: I2C, size: Dimension) -> Self {
        Self {
            address: DEFAULT_FT6X36_ADDRESS,
            i2c,
            info: None,
            #[cfg(feature = "event_process")]
            events: (None, None),
            #[cfg(feature = "event_process")]
            config: ProcessEventConfig::default(),
            orientation: Orientation::Portrait,
            size,
        }
    }

    /// Create a new Ft6x36 device with the default slave address
    ///
    /// # Arguments
    ///
    /// - `i2c` I2C bus used to communicate with the device
    /// - `size` [`Dimension`] of the device
    /// - `config`- [`ProcessEventConfig`] for the event processor
    ///
    /// # Returns
    ///
    /// - [Ft6x36 driver](Ft6x36) created
    ///
    #[cfg(feature = "event_process")]
    pub fn new_with_config(i2c: I2C, size: Dimension, config: ProcessEventConfig) -> Self {
        Self {
            address: DEFAULT_FT6X36_ADDRESS,
            i2c,
            info: None,
            events: (None, None),
            config,
            size,
            orientation: Orientation::Portrait,
        }
    }

    /// Initialize the device
    ///
    /// Currently it only gathers information on the device and initializes the
    /// [info structure of the driver](Ft6x36Info)
    ///
    pub fn init(&mut self) -> Result<(), <I2C as ErrorType>::Error> {
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

    /// Change orientation for the device
    ///
    /// # Arguments
    ///
    /// - `orientation` - set the new [`Orientation`]
    pub fn set_orientation(&mut self, orientation: Orientation) {
        self.orientation = orientation;
    }

    /// Get the full raw report of touch events
    ///
    /// # Returns
    ///
    /// - [`TouchEvent`] the full TouchEvent report
    pub fn get_touch_event(&mut self) -> Result<RawTouchEvent, <I2C as ErrorType>::Error> {
        let mut report: [u8; REPORT_SIZE] = [0; REPORT_SIZE];
        self.i2c
            .write_read(self.address, &[Reg::DeviceMode.into()], &mut report)?;

        let event: RawTouchEvent = report.into();
        Ok(event.translate_orientation(self.size, self.orientation))
    }

    /// Get the current gesture detection parameters
    /// (Currently not working)
    pub fn get_gesture_params(&mut self) -> Result<GestureParams, <I2C as ErrorType>::Error> {
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
    pub fn set_touch_threshold(&mut self, value: u8) -> Result<(), <I2C as ErrorType>::Error> {
        self.i2c
            .write(self.address, &[Reg::TouchDetectionThreshold.into(), value])
    }

    /// Set the touch filter coefficient
    ///
    /// # Arguments
    ///
    /// - `value` the touch filter coefficient
    pub fn set_touch_filter_coefficient(
        &mut self,
        value: u8,
    ) -> Result<(), <I2C as ErrorType>::Error> {
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
    pub fn set_control_mode(&mut self, value: u8) -> Result<(), <I2C as ErrorType>::Error> {
        self.i2c
            .write(self.address, &[Reg::ControlMode.into(), value])
    }

    /// Set the period used to switch from Active to Monitor mode
    ///
    /// # Arguments
    ///
    /// - `value` the switching period
    pub fn set_time_active_monitor(&mut self, value: u8) -> Result<(), <I2C as ErrorType>::Error> {
        self.i2c
            .write(self.address, &[Reg::TimeActiveMonitor.into(), value])
    }

    /// Set the report rate in Active mode
    ///
    /// # Arguments
    ///
    /// - `value` the report rate in Active mode
    pub fn set_period_active(&mut self, value: u8) -> Result<(), <I2C as ErrorType>::Error> {
        self.i2c
            .write(self.address, &[Reg::PeriodActive.into(), value])
    }

    /// Set the report rate in Monitor mode
    ///
    /// # Arguments
    ///
    /// - `value` the report rate in Monitor mode
    pub fn set_period_monitor(&mut self, value: u8) -> Result<(), <I2C as ErrorType>::Error> {
        self.i2c
            .write(self.address, &[Reg::PeriodMonitor.into(), value])
    }

    /// Set the minimum angle for gesture detection
    ///
    /// # Arguments
    ///
    /// - `value` The value of the minimum allowed angle while rotating gesture
    ///   mode
    pub fn set_gesture_minimum_angle(
        &mut self,
        value: u8,
    ) -> Result<(), <I2C as ErrorType>::Error> {
        self.i2c
            .write(self.address, &[Reg::GestRadianValue.into(), value])
    }

    /// Set the maximum offset for detecting Moving left and Moving right gestures
    ///
    /// # Arguments
    ///
    /// - `value` The value of the maximum offset for detecting Moving left and Moving right gestures
    ///   mode
    pub fn set_gesture_offset_left_right(
        &mut self,
        value: u8,
    ) -> Result<(), <I2C as ErrorType>::Error> {
        self.i2c
            .write(self.address, &[Reg::GestOffsetLeftRight.into(), value])
    }

    /// Set the maximum offset for detecting Moving up and Moving down gestures
    ///
    /// # Arguments
    ///
    /// - `value` The value of the maximum offset for detecting Moving up and Moving down gestures
    ///   mode
    pub fn set_gesture_offset_up_down(
        &mut self,
        value: u8,
    ) -> Result<(), <I2C as ErrorType>::Error> {
        self.i2c
            .write(self.address, &[Reg::GestOffsetUpDown.into(), value])
    }

    /// Set the minimum distance for detecting Moving up and Moving down gestures
    ///
    /// # Arguments
    ///
    /// - `value` The value of the minimum distance for detecting Moving up and Moving down gestures
    ///   mode
    pub fn set_gesture_distance_up_down(
        &mut self,
        value: u8,
    ) -> Result<(), <I2C as ErrorType>::Error> {
        self.i2c
            .write(self.address, &[Reg::GestDistUpDown.into(), value])
    }

    /// Set the minimum distance for detecting Moving left and Moving right gestures
    ///
    /// # Arguments
    ///
    /// - `value` The value of the minimum distance for detecting Moving left and Moving right
    ///   gestures mode
    pub fn set_gesture_distance_left_right(
        &mut self,
        value: u8,
    ) -> Result<(), <I2C as ErrorType>::Error> {
        self.i2c
            .write(self.address, &[Reg::GestDistLeftRight.into(), value])
    }

    /// Set the minimum distance for detecting zoom gestures
    ///
    /// # Arguments
    ///
    /// - `value` The value of the minimum distance for detecting zoom gestures mode
    pub fn set_gesture_distance_zoom(
        &mut self,
        value: u8,
    ) -> Result<(), <I2C as ErrorType>::Error> {
        self.i2c
            .write(self.address, &[Reg::GestDistZoom.into(), value])
    }

    /// Get device information
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
    pub fn get_diagnostics(&mut self) -> Result<Diagnostics, <I2C as ErrorType>::Error> {
        let mut buf: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[Reg::PowerMode.into()], &mut buf)?;
        let power_mode = buf[0];
        self.i2c
            .write_read(self.address, &[Reg::GMode.into()], &mut buf)?;
        let g_mode = buf[0];
        self.i2c
            .write_read(self.address, &[Reg::OperatingMode.into()], &mut buf)?;
        let state = buf[0];
        self.i2c
            .write_read(self.address, &[Reg::LibVersionH.into()], &mut buf)?;
        let lib_version_h = buf[0];
        self.i2c
            .write_read(self.address, &[Reg::LibVersionL.into()], &mut buf)?;
        let lib_version_l = buf[0];
        let lib_version: u16 = (lib_version_h as u16) << 8 | lib_version_l as u16;
        self.i2c
            .write_read(self.address, &[Reg::ControlMode.into()], &mut buf)?;
        let control_mode = buf[0];
        Ok(Diagnostics {
            power_mode,
            g_mode,
            state,
            lib_version,
            control_mode,
        })
    }

    #[cfg(feature = "event_process")]
    pub fn process_event(
        &mut self,
        time: core::time::Duration,
        event: RawTouchEvent,
    ) -> Option<TouchEvent> {
        // If there are no touch points, it means the end of gesture
        // We need to analyze it
        if event.p1.is_none() {
            match (self.events.0.take(), self.events.1.take()) {
                (Some(e1), Some(mut e2)) => {
                    if (e2.time - e1.time).le(&self.config.gesture_timing) {
                        (match (e1.event.p1, e2.event.p1) {
                            (Some(e1p1), Some(e2p1)) => process_swipe(e1p1, e2p1, &self.config),
                            _ => None,
                        })
                        .or_else(|| process_touch_points(e2.event.p1.take(), e2.event.p2.take()))
                    } else {
                        process_touch_points(e2.event.p1.take(), e2.event.p2.take())
                    }
                }
                (Some(mut e1), None) => {
                    process_touch_points(e1.event.p1.take(), e1.event.p2.take())
                }
                (_, _) => None,
            }
        } else {
            // The gesture is not terminated, we should keep track of the last event
            if self.events.0.is_some() {
                // Though if a previous event was TouchTwoPoint we should not
                // ditch it in favor of a TouchOnePoint if there are happening
                // close to each other
                match &self.events.1 {
                    Some(old_evt1) => {
                        let time_evt1 = old_evt1.time;
                        let old_evt1 = old_evt1.event;
                        if old_evt1.p2.is_none()
                            || event.p2.is_some()
                            || (time - time_evt1) > MAX_DELTA_TOUCH_EVENT
                        {
                            self.events.1 = Some(TimedRawTouchEvent { time, event })
                        }
                    }
                    None => self.events.1 = Some(TimedRawTouchEvent { time, event }),
                }
            } else {
                self.events.0 = Some(TimedRawTouchEvent { time, event });
            }
            None
        }
    }
}

#[cfg(feature = "event_process")]
fn process_touch_points(
    mut p1: Option<TouchPoint>,
    mut p2: Option<TouchPoint>,
) -> Option<TouchEvent> {
    match (p1.take(), p2.take()) {
        (Some(p1), Some(p2)) => Some(TouchEvent::TouchTwoPoint(p1, p2)),
        (Some(p1), None) => Some(TouchEvent::TouchOnePoint(p1)),
        _ => None,
    }
}

#[cfg(feature = "event_process")]
fn process_swipe(
    e1p1: TouchPoint,
    e2p1: TouchPoint,
    config: &ProcessEventConfig,
) -> Option<TouchEvent> {
    let delta_x = (e1p1.x as i16 - e2p1.x as i16).unsigned_abs();
    let delta_y = (e1p1.y as i16 - e2p1.y as i16).unsigned_abs();
    if delta_x < config.max_swipe_delta && delta_y > config.min_swipe_delta {
        if e1p1.y > e2p1.y {
            Some(TouchEvent::Swipe(
                Direction::Down,
                SwipeInfo {
                    velocity: e1p1.y - e2p1.y,
                    point: e1p1,
                },
            ))
        } else {
            Some(TouchEvent::Swipe(
                Direction::Up,
                SwipeInfo {
                    velocity: e2p1.y - e1p1.y,
                    point: e1p1,
                },
            ))
        }
    } else if delta_x > config.max_swipe_delta && delta_y < config.min_swipe_delta {
        if e1p1.x > e2p1.x {
            Some(TouchEvent::Swipe(
                Direction::Right,
                SwipeInfo {
                    velocity: e1p1.x - e2p1.x,
                    point: e1p1,
                },
            ))
        } else {
            Some(TouchEvent::Swipe(
                Direction::Left,
                SwipeInfo {
                    velocity: e2p1.x - e1p1.x,
                    point: e1p1,
                },
            ))
        }
    } else {
        None
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_raw_event_from_report_ok() {
        #[allow(clippy::unusual_byte_groupings)]
        let report: [u8; REPORT_SIZE] = [
            0x00,         // Device Mode
            0x00,         // Gesture id
            0x02,         // Touch status
            0b10_00_0000, // Touch type _ Reserved _ XH
            0x13,         // XL
            0b0000_0000,  // Touch id _ YH
            0x14,         // YL
            0x00,         // Weight
            0x00,         // Misc
            0b10_00_0000, // Touch type _ Reserved _ XH
            0x25,         // XL
            0b0000_0000,  // Touch id _ YH
            0x26,         // YL
            0x00,         // Weight
            0x00,         // Misc
        ];
        let actual: RawTouchEvent = report.into();

        let expected = RawTouchEvent {
            device_mode: DeviceMode::Working,
            gesture_id: GestureId::NoGesture,
            p1: Some(TouchPoint {
                touch_type: TouchType::Contact,
                x: 0x13,
                y: 0x14,
            }),
            p2: Some(TouchPoint {
                touch_type: TouchType::Contact,
                x: 0x25,
                y: 0x26,
            }),
        };

        assert_eq!(actual, expected);
    }

    #[test]
    fn test_raw_event_from_report_ok_high_value() {
        #[allow(clippy::unusual_byte_groupings)]
        let report: [u8; REPORT_SIZE] = [
            0x00,         // Device Mode
            0x00,         // Gesture id
            0x02,         // Touch status
            0b10_00_0001, // Touch type _ Reserved _ XH
            0x13,         // XL
            0b0000_0010,  // Touch id _ YH
            0x14,         // YL
            0x00,         // Weight
            0x00,         // Misc
            0b10_00_0100, // Touch type _ Reserved _ XH
            0x25,         // XL
            0b0000_1000,  // Touch id _ YH
            0x26,         // YL
            0x00,         // Weight
            0x00,         // Misc
        ];
        let actual: RawTouchEvent = report.into();

        let expected = RawTouchEvent {
            device_mode: DeviceMode::Working,
            gesture_id: GestureId::NoGesture,
            p1: Some(TouchPoint {
                touch_type: TouchType::Contact,
                x: 0x0113,
                y: 0x0214,
            }),
            p2: Some(TouchPoint {
                touch_type: TouchType::Contact,
                x: 0x0425,
                y: 0x0826,
            }),
        };

        assert_eq!(actual, expected);
    }

    #[test]
    fn test_process_swipe_right_ok() {
        let p1 = TouchPoint {
            x: 340,
            y: 200,
            touch_type: TouchType::Contact,
        };
        let p2 = TouchPoint {
            x: 25,
            y: 203,
            touch_type: TouchType::Contact,
        };
        let actual = process_swipe(p1, p2, &ProcessEventConfig::default());
        assert!(actual.is_some());
        let actual = actual.unwrap();

        let expected = TouchEvent::Swipe(
            Direction::Right,
            SwipeInfo {
                velocity: 315,
                point: TouchPoint {
                    x: 340,
                    y: 200,
                    touch_type: TouchType::Contact,
                },
            },
        );

        assert_eq!(actual, expected);
    }

    #[test]
    fn test_process_swipe_left_ok() {
        let p1 = TouchPoint {
            x: 231,
            y: 200,
            touch_type: TouchType::Contact,
        };
        let p2 = TouchPoint {
            x: 334,
            y: 197,
            touch_type: TouchType::Contact,
        };
        let actual = process_swipe(p1, p2, &ProcessEventConfig::default());
        assert!(actual.is_some());
        let actual = actual.unwrap();

        let expected = TouchEvent::Swipe(
            Direction::Left,
            SwipeInfo {
                velocity: 103,
                point: TouchPoint {
                    x: 231,
                    y: 200,
                    touch_type: TouchType::Contact,
                },
            },
        );

        assert_eq!(actual, expected);
    }

    #[test]
    fn test_process_swipe_up_ok() {
        let p1 = TouchPoint {
            x: 231,
            y: 200,
            touch_type: TouchType::Contact,
        };
        let p2 = TouchPoint {
            x: 234,
            y: 302,
            touch_type: TouchType::Contact,
        };
        let actual = process_swipe(p1, p2, &ProcessEventConfig::default());
        assert!(actual.is_some());
        let actual = actual.unwrap();

        let expected = TouchEvent::Swipe(
            Direction::Up,
            SwipeInfo {
                velocity: 102,
                point: TouchPoint {
                    x: 231,
                    y: 200,
                    touch_type: TouchType::Contact,
                },
            },
        );

        assert_eq!(actual, expected);
    }

    #[test]
    fn test_process_swipe_down_ok() {
        let p1 = TouchPoint {
            x: 231,
            y: 200,
            touch_type: TouchType::Contact,
        };
        let p2 = TouchPoint {
            x: 225,
            y: 25,
            touch_type: TouchType::Contact,
        };
        let actual = process_swipe(p1, p2, &ProcessEventConfig::default());
        assert!(actual.is_some());
        let actual = actual.unwrap();

        let expected = TouchEvent::Swipe(
            Direction::Down,
            SwipeInfo {
                velocity: 175,
                point: TouchPoint {
                    x: 231,
                    y: 200,
                    touch_type: TouchType::Contact,
                },
            },
        );

        assert_eq!(actual, expected);
    }
}
