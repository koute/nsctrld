use {
    derive_more::{
        BitAnd,
        BitOr,
        Display
    },
    speedy::{
        Readable
    },
    std::{
        fmt,
        time::{
            Duration,
            Instant
        }
    }
};

#[macro_use]
extern crate log;

mod usb;

#[derive(Debug)]
enum SubcommandStatus {
    Ack {
        data_type: u8
    },
    Nack
}

impl< 'a, C > speedy::Readable< 'a, C > for SubcommandStatus where C: speedy::Context {
    fn read_from< R >( reader: &mut R ) -> Result< Self, C::Error > where R: speedy::Reader< 'a, C > {
        let value = reader.read_u8()?;
        let status = if value & 0b1000_0000 != 0 {
            SubcommandStatus::Ack {
                data_type: value & 0b0111_1111
            }
        } else {
            SubcommandStatus::Nack
        };

        Ok( status )
    }
}

#[derive(Debug)]
enum SubcommandData {
    DeviceInfo( DeviceInfo ),
    Unknown( Vec< u8 > )
}

#[derive(Debug)]
enum SubcommandResult {
    Ack,
    AckWithData( SubcommandData ),
    Nack
}

#[derive(Debug)]
struct SubcommandReply {
    subcommand: u8,
    result: SubcommandResult
}

impl< 'a, C > speedy::Readable< 'a, C > for SubcommandReply where C: speedy::Context {
    fn read_from< R >( reader: &mut R ) -> Result< Self, C::Error > where R: speedy::Reader< 'a, C > {
        let status = reader.read_u8()?;
        let subcommand = reader.read_u8()?;

        let result = if status & 0b1000_0000 == 0 {
            SubcommandResult::Nack
        } else {
            let data_type = status & 0b0111_1111;
            match data_type {
                0 => SubcommandResult::Ack,
                2 => SubcommandResult::AckWithData( SubcommandData::DeviceInfo( reader.read_value()? ) ),
                _ => SubcommandResult::AckWithData( SubcommandData::Unknown( reader.read_vec( 35 )? ) )
            }
        };

        let reply = SubcommandReply {
            subcommand,
            result
        };

        Ok( reply )
    }
}

#[derive(Readable, Debug)]
struct InputReport {
    timestamp: u8,
    misc: u8,
    buttons: [u8; 3],
    left_stick: [u8; 3],
    right_stick: [u8; 3],
    unk1: u8
}

#[derive(Readable, Debug)]
#[speedy(tag_type = u8)]
enum DeviceType {
    #[speedy(tag = 1)]
    LeftJoyCon,
    #[speedy(tag = 2)]
    RightJoyCon,
    #[speedy(tag = 3)]
    ProController,
}

#[derive(Readable, Debug)]
struct MacAddress( [u8; 6] );

impl fmt::Display for MacAddress {
    fn fmt( &self, fmt: &mut fmt::Formatter ) -> fmt::Result {
        write!(
            fmt,
            "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            self.0[ 0 ], self.0[ 1 ], self.0[ 2 ], self.0[ 3 ], self.0[ 4 ], self.0[ 5 ]
        )
    }
}

#[derive(Readable, Debug)]
struct DeviceInfo {
    firmware_major: u8,
    firmware_minor: u8,
    kind: DeviceType,
    unk1: u8,
    mac_address: MacAddress,
    unk2: u8,
    unk3: u8
}

#[derive(Readable, Debug)]
#[speedy(tag_type = u8)]
enum Packet {
    #[speedy(tag = 0x00)]
    Empty,
    #[speedy(tag = 0x81)]
    Status {
        ty: u8,
        payload: [u8; 8]
    },
    #[speedy(tag = 0x30)]
    Input( InputReport ),
    #[speedy(tag = 0x21)]
    InputWithSubcommandReply {
        report: InputReport,
        reply: SubcommandReply
    }
}

impl Packet {
    fn parse( buffer: &[u8] ) -> Result< Self, Error > {
        match Packet::read_from_buffer( buffer ).map_err( Error::CannotParseReceivedData ) {
            Ok( packet ) => Ok( packet ),
            Err( error ) => {
                error!( "Failed to parse the following packet: {:?}", buffer );
                return Err( error );
            }
        }
    }
}

#[derive(Debug, Display)]
pub enum Error {
    CannotOpenDevice( crate::usb::Error ),
    CannotCheckIfKernelDriverIsActive( crate::usb::Error ),
    CannotDetachKernelDriver( crate::usb::Error ),
    CannotCheckActiveConfiguration( crate::usb::Error ),
    CannotSetActiveConfiguration( crate::usb::Error ),
    CannotClaimInterface( crate::usb::Error ),
    CannotInitiateHandshake( crate::usb::Error ),
    CannotReadData( crate::usb::Error ),
    CannotParseReceivedData( speedy::Error ),
    CannotWriteData( crate::usb::Error ),
    TransferFailed( crate::usb::TransferError ),
    PartialWrite,
    HandshakeFailed
}

impl std::error::Error for Error {}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, BitAnd, BitOr)]
pub struct Led( u8 );

impl Led {
    pub const NONE  : Led = Led( 0b0000_0000 );
    pub const FIRST : Led = Led( 0b0000_0001 );
    pub const SECOND: Led = Led( 0b0000_0010 );
    pub const THIRD : Led = Led( 0b0000_0100 );
    pub const FOURTH: Led = Led( 0b0000_1000 );
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, BitAnd, BitOr)]
pub struct Button( u32 );

impl Button {
    pub const A      : Button = Button( 0b00000000_00000000_00001000 );
    pub const B      : Button = Button( 0b00000000_00000000_00000100 );
    pub const X      : Button = Button( 0b00000000_00000000_00000010 );
    pub const Y      : Button = Button( 0b00000000_00000000_00000001 );
    pub const LEFT   : Button = Button( 0b00001000_00000000_00000000 );
    pub const RIGHT  : Button = Button( 0b00000100_00000000_00000000 );
    pub const UP     : Button = Button( 0b00000010_00000000_00000000 );
    pub const DOWN   : Button = Button( 0b00000001_00000000_00000000 );
    pub const ZL     : Button = Button( 0b10000000_00000000_00000000 );
    pub const L      : Button = Button( 0b01000000_00000000_00000000 );
    pub const ZR     : Button = Button( 0b00000000_00000000_10000000 );
    pub const R      : Button = Button( 0b00000000_00000000_01000000 );
    pub const LS     : Button = Button( 0b00000000_00001000_00000000 );
    pub const RS     : Button = Button( 0b00000000_00000100_00000000 );
    pub const PLUS   : Button = Button( 0b00000000_00000010_00000000 );
    pub const MINUS  : Button = Button( 0b00000000_00000001_00000000 );
    pub const CAPTURE: Button = Button( 0b00000000_00100000_00000000 );
    pub const HOME   : Button = Button( 0b00000000_00010000_00000000 );

    pub fn is_pressed( self, button: Button ) -> bool {
        (self.0 & button.0) == button.0
    }

    pub fn iter_changed( self, old_state: Button ) -> impl Iterator< Item = (Button, bool) > {
        let xor = self.0 ^ old_state.0;
        (0..24)
            .map( |n_bit| 1 << n_bit )
            .filter( move |mask| xor & mask != 0 )
            .map( move |mask| (Button( mask ), self.0 & mask != 0) )
    }
}

impl fmt::Debug for Button {
    #[allow(unused_assignments)]
    fn fmt( &self, fmt: &mut fmt::Formatter ) -> fmt::Result {
        let mut is_empty = true;
        macro_rules! append {
            ($($name:ident),+) => {
                $(
                    if self.is_pressed( Button::$name ) {
                        if !is_empty {
                            write!( fmt, " | " )?;
                        }

                        write!( fmt, stringify!( $name ) )?;
                        is_empty = false;
                    }
                )+
            }
        }

        append! { A, B, X, Y, LEFT, RIGHT, UP, DOWN, ZL, ZR, L, R, LS, RS, MINUS, PLUS, HOME, CAPTURE }
        Ok(())
    }
}

// Source of the protocol info:
//   Initialization:
//       https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/USB-HID-Notes.md
//   Rumble, LEDs:
//       https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_notes.md
//       https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_subcommands_notes.md
//       https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/issues/11
//       https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/rumble_data_table.md

/// A structure containing the haptic feedback configuration for the controller.
///
/// The haptic feedback motor used for vibrations has has two independently controlled
/// oscillation directions - a low frequency one with a 160Hz resonance frequency,
/// and a high frequency one with a 320Hz resonance frequency.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct RumbleData( [u8; 4] );

impl Default for RumbleData {
    fn default() -> Self {
        RumbleData::new_with_raw( 0x40, 0, 0x40, 0 )
    }
}

#[test]
fn test_rumble_data_default() {
    assert_eq!(
        RumbleData::default().0,
        [ 0x00, 0x01, 0x40, 0x40 ]
    )
}

impl RumbleData {
    /// Creates a new rumble configuration with vibrations disabled.
    pub fn disabled() -> Self {
        Self::default()
    }

    /// Creates a new rumble configuration with a given raw frequency and amplitude
    /// for each of the oscillation channels.
    ///
    /// The valid range of `low_channel_f` and `high_channel_f` is between 1 and 127.
    /// The valid range of `low_channel_a` and `high_channel_a` is between 0 and 100.
    /// Values outside of those ranges will be clamped.
    pub fn new_with_raw(
        mut low_channel_f: u8,
        mut low_channel_a: u8,
        mut high_channel_f: u8,
        mut high_channel_a: u8
    ) -> Self {
        if low_channel_f < 1 {
            low_channel_f = 1;
        }

        if low_channel_f > 127 {
            low_channel_f = 127;
        }

        if high_channel_f < 1 {
            high_channel_f = 1;
        }

        if high_channel_f > 127 {
            high_channel_f = 127;
        }

        // You can go higher, however this is the maximum safe amplitude.
        if low_channel_a > 100 {
            low_channel_a = 100;
        }

        if high_channel_a > 100 {
            high_channel_a = 100;
        }

        // BIT PATTERN:
        // aaaaaa00 bbbbbbba dccccccc 01dddddd
        //
        // aaaaaaa High channel Frequency
        // bbbbbbb High channel Amplitude
        // ccccccc Low channel Frequency
        // ddddddd Low channel Amplitude

        let data = [
            high_channel_f << 2,
            (high_channel_f >> 6) | (high_channel_a << 1),
            ((low_channel_a & 0b1) << 7) | low_channel_f,
            0b01000000 | (low_channel_a >> 1)
        ];

        RumbleData( data )
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct StickPosition {
    pub x: f32,
    pub y: f32
}

impl StickPosition {
    pub fn new() -> Self {
        StickPosition::default()
    }
}

struct StickCalibration {
    min_x: u16,
    max_x: u16,
    min_y: u16,
    max_y: u16
}

impl StickCalibration {
    fn new() -> Self {
        StickCalibration {
            min_x: 700,
            max_x: 3300,
            min_y: 700,
            max_y: 3300
        }
    }

    fn adjust( &mut self, x: u16, y: u16 ) {
        if x < self.min_x { self.min_x = x }
        if x > self.max_x { self.max_x = x }
        if y < self.min_y { self.min_y = y }
        if y > self.max_y { self.max_y = y }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum Sign {
    Zero,
    Positive,
    Negative
}

impl Sign {
    fn from_value( value: f32 ) -> Self {
        if value > std::f32::EPSILON {
            Sign::Positive
        } else if value < -std::f32::EPSILON {
            Sign::Negative
        } else {
            Sign::Zero
        }
    }
}

pub struct Controller {
    handle: crate::usb::DeviceHandle,
    increased_rate: bool,
    initialized: bool,
    device_info_initialized: bool,
    packet_counter: u8,
    button_state: Button,
    led_state: Led,
    pending_led_state: Led,
    rumble_left: RumbleData,
    rumble_right: RumbleData,
    rumble_changed: bool,
    last_subcommand_timestamp: Instant,
    subcommand_in_flight: bool,
    inbound_transfer: usb::Transfer,
    outbound_transfers: Vec< usb::Transfer >,
    firmware_major: u8,
    firmware_minor: u8,
    mac_address: MacAddress,
    rx: flume::Receiver< crate::usb::TransferLock >,
    tx: flume::Sender< crate::usb::TransferLock >,
    read_pending: bool,
    write_pending: usize,
    packet_queue: Vec< Packet >,

    left_calibration: StickCalibration,
    right_calibration: StickCalibration,

    left_stick: StickPosition,
    left_stick_unfiltered: [StickPosition; 2],
    right_stick: StickPosition,
    right_stick_unfiltered: [StickPosition; 2]
}

const TIMEOUT: Duration = Duration::from_secs( 1 );

impl Controller {
    pub fn open( device: &Device ) -> Result< Self, Error > {
        use Error::*;

        let mut handle = device.open().map_err( CannotOpenDevice )?;
        if handle.kernel_driver_active( 0 ).map_err( CannotCheckIfKernelDriverIsActive )? {
            handle.detach_kernel_driver( 0 ).map_err( CannotDetachKernelDriver )?;
        }

        if handle.active_configuration().map_err( CannotCheckActiveConfiguration )? != 1 {
            handle.set_active_configuration( 1 ).map_err( CannotSetActiveConfiguration )?;
        }

        handle.claim_interface( 0 ).map_err( CannotClaimInterface )?;
        handle.write_interrupt( 1, &[ 0x80, 0x01 ], TIMEOUT ).map_err( CannotInitiateHandshake )?;

        let (tx, rx) = flume::bounded( 4 );
        let mut controller = Controller {
            handle,
            increased_rate: false,
            initialized: false,
            device_info_initialized: false,
            packet_counter: 0,
            button_state: Button( 0 ),
            pending_led_state: Led( 0 ),
            led_state: Led( 0 ),
            rumble_left: RumbleData::disabled(),
            rumble_right: RumbleData::disabled(),
            rumble_changed: false,
            last_subcommand_timestamp: Instant::now(),
            subcommand_in_flight: false,
            inbound_transfer: usb::Transfer::new(),
            outbound_transfers: Vec::new(),
            firmware_major: 0,
            firmware_minor: 0,
            mac_address: MacAddress( [0; 6] ),
            rx,
            tx: tx.clone(),
            read_pending: false,
            write_pending: 0,
            packet_queue: Vec::new(),

            left_calibration: StickCalibration::new(),
            right_calibration: StickCalibration::new(),

            left_stick: Default::default(),
            left_stick_unfiltered: Default::default(),
            right_stick: Default::default(),
            right_stick_unfiltered: Default::default()
        };

        let mut inbound_transfer = controller.inbound_transfer.lock();
        inbound_transfer.buffer_mut().resize( 64, 0 );
        inbound_transfer.set_completion_sender( tx );
        std::mem::drop( inbound_transfer );

        let timestamp = Instant::now();
        while timestamp.elapsed() <= Duration::from_millis( 5000 ) {
            controller.poll_with_timeout( Duration::from_millis( 100 ) )?;
            if controller.initialized {
                break;
            }
        }

        if !controller.initialized {
            return Err( HandshakeFailed );
        }

        controller.send_request_device_info()?;

        let timestamp = Instant::now();
        while timestamp.elapsed() <= Duration::from_millis( 500 ) && !controller.device_info_initialized {
            controller.poll_with_timeout( Duration::from_millis( 100 ) )?;
        }

        controller.send_change_led( Led::NONE )?;
        controller.wait_for_command_finish()?;

        controller.send_change_home_light( 0 )?;
        controller.wait_for_command_finish()?;

        Ok( controller )
    }

    fn wait_for_command_finish( &mut self ) -> Result< (), Error > {
        let timestamp = Instant::now();
        while timestamp.elapsed() <= Duration::from_millis( 500 ) && self.subcommand_in_flight {
            self.poll_with_timeout( Duration::from_millis( 100 ) )?;
        }

        Ok(())
    }

    fn send_subcommand( &mut self, command: u8, data: &[u8] ) -> Result< (), Error > {
        // If we send more than one subcommand at the same time
        // the controller seems to get confused and will
        // either send us only one reply, or none at all.
        assert!( !self.subcommand_in_flight );

        let mut buffer = [0; 0x40];
        buffer[ 0 ] = 1;
        buffer[ 1 ] = self.packet_counter;
        self.packet_counter = (self.packet_counter + 1) % 0x10;

        buffer[ 2..6 ].copy_from_slice( &self.rumble_left.0 );
        buffer[ 6..10 ].copy_from_slice( &self.rumble_right.0 );
        buffer[ 10 ] = command;
        buffer[ 11..11 + data.len() ].copy_from_slice( data );
        let result = self.write( &buffer );
        if result.is_ok() {
            self.subcommand_in_flight = true;
            self.rumble_changed = false;
            self.last_subcommand_timestamp = Instant::now();
        }
        result
    }

    fn send_request_device_info( &mut self ) -> Result< (), Error > {
        self.send_subcommand( 0x02, &[] )
    }

    fn send_rumble( &mut self ) -> Result< (), Error > {
        // We don't use the rumble-only message here since
        // that one doesn't trigger a reply, and we need the
        // replies to know when to send the next message.
        self.send_subcommand( 0x00, &[] )
    }

    fn send_change_led( &mut self, mask: Led ) -> Result< (), Error > {
        self.send_subcommand( 0x30, &[ mask.0 ] )
    }

    fn send_change_home_light( &mut self, intensity: u8 ) -> Result< (), Error >{
        let intensity = intensity / 16;
        assert!( intensity <= 0b0000_1111 );
        self.send_subcommand( 0x38, &[ 0b0000_0001, intensity << 4 | 0b0000, intensity << 4, 0xff, 0xff, 0xff ] )
    }

    pub fn set_led( &mut self, mask: Led ) {
        if self.led_state == mask {
            return;
        }

        self.led_state = mask;
    }

    pub fn set_rumble( &mut self, data: RumbleData ) {
        self.set_rumble_separate( data, data )
    }

    pub fn set_rumble_separate( &mut self, left: RumbleData, right: RumbleData ) {
        if left == self.rumble_left && right == self.rumble_right {
            return;
        }

        self.rumble_left = left;
        self.rumble_right = right;

        if left != right {
            debug!( "New rumble:" );
            debug!(
                "  L: {:02x} {:02x} {:02x} {:02x} ({:08b} {:08b} {:08b} {:08b})",
                left.0[0], left.0[1], left.0[2], left.0[3],
                left.0[0], left.0[1], left.0[2], left.0[3]
            );
            debug!(
                "  R: {:02x} {:02x} {:02x} {:02x} ({:08b} {:08b} {:08b} {:08b})",
                right.0[0], right.0[1], right.0[2], right.0[3],
                right.0[0], right.0[1], right.0[2], right.0[3]
            );
        } else {
            debug!(
                "New rumble: {:02x} {:02x} {:02x} {:02x} ({:08b} {:08b} {:08b} {:08b})",
                left.0[0], left.0[1], left.0[2], left.0[3],
                left.0[0], left.0[1], left.0[2], left.0[3]
            );
        }

        self.rumble_changed = true;
    }

    fn write( &mut self, data: &[u8] ) -> Result< (), Error > {
        let transfer = if let Some( transfer ) = self.outbound_transfers.pop() {
            transfer
        } else {
            let transfer = usb::Transfer::new();
            let mut locked = transfer.lock();
            locked.buffer_mut().reserve( 64 );
            locked.set_completion_sender( self.tx.clone() );
            transfer
        };

        let mut transfer = transfer.try_lock().unwrap();
        self.write_pending += 1;

        transfer.set_data( data );
        transfer.submit_interrupt( &self.handle, 1, TIMEOUT ).map_err( Error::CannotWriteData )
    }

    pub fn button_state( &self ) -> Button {
        self.button_state
    }

    pub fn left_stick( &self ) -> StickPosition {
        let mut stick = self.left_stick;
        // Invert, since everyone on Linux treats positive values as being down.
        stick.y *= -1.0;
        stick
    }

    pub fn right_stick( &self ) -> StickPosition {
        let mut stick = self.right_stick;
        stick.y *= -1.0;
        stick
    }

    pub fn firmware_version( &self ) -> (u8, u8) {
        (self.firmware_major, self.firmware_minor)
    }

    fn handle_input_report( &mut self, report: InputReport ) {
        let buttons = report.buttons;
        let button_state = Button( buttons[ 0 ] as u32 | (buttons[ 1 ] as u32) << 8 | (buttons[ 2 ] as u32) << 16 );
        if self.button_state != button_state {
            debug!( "Raw button state: {:08b} {:08b} {:08b}", buttons[ 2 ], buttons[ 1 ], buttons[ 0 ] );
            debug!( "Button state: {:?}", button_state );
            self.button_state = button_state;
        }

        fn get_stick( data: &[u8; 3] ) -> (u16, u16) {
            let x = data[0] as u16 | ((data[1] as u16 & 0xF) << 8);
            let y = (data[1] as u16 >> 4) | ((data[2] as u16) << 4);
            (x, y)
        }

        fn convert_stick( data: &[u8; 3], calibration: &mut StickCalibration ) -> StickPosition {
            // The proper range (i.e. `max - min`) for the analog sticks are ~3200, so ~1600 either way.
            // The proper center is around ~2000, but can be off by ~300.
            let stick = get_stick( data );

            // Automatically calibrate as we go.
            calibration.adjust( stick.0, stick.1 );

            let target_range = 3100;
            let range_x = calibration.max_x - calibration.min_x;
            let range_y = calibration.max_y - calibration.min_y;
            let missing_range_x = std::cmp::max( 0, target_range as i32 - range_x as i32 ) as f32 / target_range as f32;
            let missing_range_y = std::cmp::max( 0, target_range as i32 - range_y as i32 ) as f32 / target_range as f32;

            fn autocenter_and_normalize( value: i32, min: i32, max: i32 ) -> f32 {
                let a = min + 1500;
                let b = max - 1500;
                let c = (a + b) / 2;
                let range_plus = std::cmp::min( max - c, 1600 );
                let range_minus = std::cmp::min( c - min, 1600 );

                if value > c {
                    (value as f32 - c as f32) / range_plus as f32
                } else {
                    (value as f32 - c as f32) / range_minus as f32
                }
            }

            let mut stick = {
                let (x, y) = stick;
                let x = autocenter_and_normalize( x as i32, calibration.min_x as i32, calibration.max_x as i32 );
                let y = autocenter_and_normalize( y as i32, calibration.min_y as i32, calibration.max_y as i32 );
                StickPosition { x, y }
            };

            let mut distance = (stick.x * stick.x + stick.y * stick.y).sqrt();
            let extra_deadzone = if missing_range_x > missing_range_y { missing_range_x } else { missing_range_y };

            let deadzone = 0.13;
            let upper_cutoff = 0.07;
            let rescale = 1.01;

            let deadzone = deadzone + extra_deadzone;
            if distance < deadzone {
                stick.x = 0.0;
                stick.y = 0.0;
            } else {
                // Rescale the non-deadzone area slightly so that we start from 0.0, and hit 1.0 just before max tilt.
                let dx = stick.x / distance;
                let dy = stick.y / distance;
                if distance >= (1.0 - upper_cutoff) {
                    distance = 1.0;
                } else {
                    distance = (distance - deadzone) / (1.0 - deadzone - upper_cutoff);
                }

                stick.x = dx * distance;
                stick.y = dy * distance;
            }

            // Rescale a little so we'll hit 1.0 at max tilt.
            stick.x *= rescale;
            stick.y *= rescale;

            if stick.x >= 1.0 { stick.x = 1.0; }
            if stick.x <= -1.0 { stick.x = -1.0; }
            if stick.y >= 1.0 { stick.y = 1.0; }
            if stick.y <= -1.0 { stick.y = -1.0; }

            stick
        }

        let left_stick = convert_stick( &report.left_stick, &mut self.left_calibration );
        let right_stick = convert_stick( &report.right_stick, &mut self.right_calibration );

        fn cmp( old: f32, new: f32, old_unfiltered: [f32; 2] ) -> bool {
            let threshold = 0.008;
            let is_above_threshold =
                (old == 0.0 && new != 0.0) ||
                (old != 0.0 && new == 0.0) ||
                (old == 1.0 && new != 1.0) ||
                (old != 1.0 && new == 1.0) ||
                (old == -1.0 && new != -1.0) ||
                (old != -1.0 && new == -1.0) ||
                (old - new).abs() >= threshold;

            if !is_above_threshold {
                // If the value didn't change much - do not update, it's just noise.
                return false;
            }

            // If we tilt a stick as far as it can go and suddenly let go
            // the hardware will generate something like this:
            //    1, 0.5, -0.1, 0
            //
            // I assume this happens because the analog stick is basically a spring,
            // so when it starts to return to its rest position it overshoots the center
            // a little due to the mechanical momentum it gains while doing so.
            //
            // We do not really want that negative value in there, so we filter it out here.
            let new_sign = Sign::from_value( new );
            let old_sign_p = Sign::from_value( old_unfiltered[0] );
            let old_sign_pp = Sign::from_value( old_unfiltered[1] );
            (new_sign == old_sign_p && new_sign == old_sign_pp) ||
            (old_sign_p == Sign::Zero && old_sign_pp == Sign::Zero)
        }

        if cmp( self.left_stick.x, left_stick.x, [self.left_stick_unfiltered[0].x, self.left_stick_unfiltered[1].x] ) {
            self.left_stick.x = left_stick.x;
        }

        if cmp( self.left_stick.y, left_stick.y, [self.left_stick_unfiltered[0].y, self.left_stick_unfiltered[1].y] ) {
            self.left_stick.y = left_stick.y;
        }

        if cmp( self.right_stick.x, right_stick.x, [self.right_stick_unfiltered[0].x, self.right_stick_unfiltered[1].x] ) {
            self.right_stick.x = right_stick.x;
        }

        if cmp( self.right_stick.y, right_stick.y, [self.right_stick_unfiltered[0].y, self.right_stick_unfiltered[1].y] ) {
            self.right_stick.y = right_stick.y;
        }

        self.left_stick_unfiltered[1] = self.left_stick_unfiltered[0];
        self.left_stick_unfiltered[0] = left_stick;
        self.right_stick_unfiltered[1] = self.right_stick_unfiltered[0];
        self.right_stick_unfiltered[0] = right_stick;
    }

    fn handle_subcommand_reply( &mut self, reply: SubcommandReply ) {
        debug!( "Handling subcommand reply: {:#?}", reply );
        self.subcommand_in_flight = false;
        match reply.subcommand {
            0x02 => {
                match reply.result {
                    SubcommandResult::AckWithData( SubcommandData::DeviceInfo( info ) ) => {
                        debug!( "Controller firmware version: {}.{}", info.firmware_major, info.firmware_minor );
                        debug!( "Controller MAC address: {}", info.mac_address );
                        self.firmware_major = info.firmware_major;
                        self.firmware_minor = info.firmware_minor;
                        self.mac_address = info.mac_address;
                        self.device_info_initialized = true;
                    },
                    _ => {}
                }
            },
            _ => {}
        }
    }

    fn handle_packet( &mut self, packet: Packet ) -> Result< (), Error > {
        trace!( "Received: {:?}", packet );
        match packet {
            Packet::Input( report ) => {
                self.handle_input_report( report );
                if !self.initialized {
                    trace!( "Controller is initialized!" );
                    self.initialized = true;
                }
            },
            Packet::InputWithSubcommandReply { report, reply } => {
                self.handle_input_report( report );
                self.handle_subcommand_reply( reply );
            },
            // These three are only used during initialization.
            Packet::Status { ty, .. } if ty == 0x01 => {
                // Send a handshake.
                trace!( "Sending handshake..." );
                self.write( &[ 0x80, 0x02 ] )?;
            },
            Packet::Status { ty, .. } if ty == 0x02 => {
                if !self.increased_rate {
                    trace!( "Increasing the baudrate..." );
                    // Switch the baudrate to 3Mbit.
                    self.write( &[ 0x80, 0x03 ] )?;
                } else {
                    trace!( "Enabling USB HID mode..." );
                    // Force the controller to talk over USB HID without any timeouts.
                    // This is required for the controller to not timeout and switch to Bluetooth.
                    self.write( &[ 0x80, 0x04 ] )?;
                }
            },
            Packet::Status { ty, .. } if ty == 0x03 => {
                trace!( "Sending a second handshake..." );
                self.increased_rate = true;
                // Another handshake is necessary.
                self.write( &[ 0x80, 0x02 ] )?;
            },
            _ => {}
        }

        Ok(())
    }

    fn process_finished_transfer( &mut self, transfer: crate::usb::TransferLock ) -> Result< (), Error > {
        if transfer == self.inbound_transfer {
            self.read_pending = false;
            if let Some( error ) = transfer.error() {
                return Err( Error::TransferFailed( error ) );
            }

            let length = transfer.bytes_transferred();
            let packet = Packet::parse( &transfer.buffer()[ ..length ] )?;
            self.packet_queue.push( packet );
        } else {
            self.write_pending -= 1;
            if let Some( error ) = transfer.error() {
                self.outbound_transfers.push( transfer.unlock() );
                return Err( Error::TransferFailed( error ) );
            }

            if transfer.bytes_transferred() != transfer.buffer().len() {
                self.outbound_transfers.push( transfer.unlock() );
                return Err( Error::PartialWrite );
            }

            self.outbound_transfers.push( transfer.unlock() );
        }

        Ok(())
    }

    pub fn poll( &mut self ) -> Result< (), Error > {
        self.poll_with_timeout( Duration::from_millis( 15 ) )
    }

    pub fn poll_with_timeout( &mut self, timeout: Duration ) -> Result< (), Error > {
        loop {
            match self.rx.try_recv() {
                Ok( transfer ) => {
                    self.process_finished_transfer( transfer )?;
                },
                Err( flume::TryRecvError::Empty ) => break,
                Err( error ) => panic!( "try_recv() failed: {:?}", error )
            };
        }

        if !self.read_pending {
            let transfer = self.inbound_transfer.try_lock().expect( "deadlock while locking the inbound transfer object" );
            transfer.submit_interrupt( &self.handle, 0x81, TIMEOUT ).map_err( Error::CannotReadData )?;
            self.read_pending = true;
        }

        while !self.packet_queue.is_empty() {
            let packet = self.packet_queue.remove( 0 );
            self.handle_packet( packet )?;
        }

        if self.write_pending == 0 {
            let elapsed = self.last_subcommand_timestamp.elapsed();
            if self.subcommand_in_flight && elapsed >= Duration::from_millis( 30 ) {
                self.subcommand_in_flight = false;
            }

            if !self.subcommand_in_flight && self.led_state != self.pending_led_state {
                self.pending_led_state = self.led_state;
                self.send_change_led( self.led_state )?;
            }

            if !self.subcommand_in_flight && (elapsed >= Duration::from_millis( 1000 ) || self.rumble_changed) {
                self.send_rumble()?;
            }
        }

        if self.read_pending || self.write_pending > 0 {
            let transfer = match self.rx.recv_timeout( timeout ) {
                Ok( transfer ) => transfer,
                Err( flume::RecvTimeoutError::Timeout ) => return Ok(()),
                Err( error ) => panic!( "recv_timeout() failed: {:?}", error )
            };

            self.process_finished_transfer( transfer )?;
        }

        Ok(())
    }
}

pub struct UsbContext( crate::usb::Context );

impl UsbContext {
    pub fn new() -> Result< Self, crate::usb::Error > {
        Ok( Self( crate::usb::Context::new()? ) )
    }
}

/// Searches the system for any connected Nintendo Pro Controllers.
pub fn find_devices( usb: &UsbContext ) -> Result< impl Iterator< Item = Device >, crate::usb::Error > {
    let mut devices = usb.0.devices()?;
    devices.retain( |device| device.vendor_id() == 0x057e && device.product_id() == 0x2009 );
    Ok( devices.into_iter() )
}

pub fn hotplug_events( usb: &UsbContext ) -> Result< crate::usb::HotplugStream, crate::usb::Error > {
    usb.0.hotplug_events( Some( 0x057e ), Some( 0x2009 ), None )
}

pub use crate::usb::{Device, HotplugEvent, HotplugStream};
pub use crate::usb::Error as UsbError;
