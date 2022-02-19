#![no_std]

use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Write, WriteRead},
};

use num_enum::{FromPrimitive, IntoPrimitive};

const DEFAULT_FT6X36_ADDRESS: u8 = 0x38;
const REPORT_SIZE: usize = 0x0f;

pub struct Ft6x36<I2C> {
    address: u8,
    i2c: I2C,
    info: Option<Ft6x36Info>,
}

pub struct Point {
    pub x: u8,
    pub y: u8,
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

/*
pub enum TouchEvent {
    NoEvent,
    TouchOnePoint(Point),
    TouchTwoPoint(Point, Point),
    Swipe(Direction, SwipeInfo),
    Zoom(Zoom),
}
*/

#[derive(Debug)]
pub struct TouchEvent {
    device_mode: u8,
    gesture_id: u8,
    touch_device_status: u8,
    p1xh: u8,
    p1xl: u8,
    p1yh: u8,
    p1yl: u8,
    p1weight: u8,
    p1misc: u8,
    p2xh: u8,
    p2xl: u8,
    p2yh: u8,
    p2yl: u8,
    p2weight: u8,
    p2misc: u8,
}

macro_rules! get_offset {
    ($first:expr, $relative_offset:expr) => {
        ($relative_offset as u8 - $first as u8) as usize
    };
}

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
    ChipId = 0xA3,
    FirmwareId = 0xA6,
    PanelId = 0xA8,
    ReleaseCode = 0xAF,
    OperatingMode = 0xBC,
}

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

#[repr(u8)]
#[derive(Clone, Copy, Debug, IntoPrimitive, FromPrimitive)]
pub enum ChipId {
    #[default]
    Unknown,
    Ft6206 = 0x06,
    Ft6236 = 0x36,
    Ft6236u = 0x64,
}

#[derive(Clone, Copy, Debug)]
pub struct Ft6x36Info {
    chip_id: ChipId,
    firmware_id: u8,
    panel_id: u8,
    release_code: u8,
}

impl<I2C, E> Ft6x36<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            address: DEFAULT_FT6X36_ADDRESS,
            i2c,
            info: None,
        }
    }

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
        Ok(())
    }

    pub fn get_touch_event(&mut self) -> Result<TouchEvent, E> {
        let mut report: [u8; REPORT_SIZE] = [0; REPORT_SIZE];
        self.i2c
            .write_read(self.address, &[Reg::DeviceMode.into()], &mut report)?;

        Ok(TouchEvent {
            device_mode: report[0],
            gesture_id: report[1],
            touch_device_status: report[2],
            p1xh: report[3],
            p1xl: report[4],
            p1yh: report[5],
            p1yl: report[6],
            p1weight: report[7],
            p1misc: report[8],
            p2xh: report[9],
            p2xl: report[0xa],
            p2yh: report[0xb],
            p2yl: report[0xc],
            p2weight: report[0xd],
            p2misc: report[0xe],
        })
    }

    pub fn get_info(&self) -> Option<Ft6x36Info> {
        self.info
    }
}
