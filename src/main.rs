use {
    linux_input::{
        AbsoluteAxis,
        AbsoluteAxisBit,
        Bus,
        DeviceId,
        EventBit,
        ForceFeedback,
        ForceFeedbackEffectKind,
        ForceFeedbackRequest,
        InputEvent,
        InputEventBody,
        Key
    },
    std::{
        collections::{
            HashMap,
            HashSet
        },
        path::{
            Path
        },
        sync::{
            atomic::{
                AtomicBool,
                Ordering
            }
        }
    }
};

static RUNNING: AtomicBool = AtomicBool::new( true );

fn is_accessible( path: &Path ) -> bool {
    use std::os::unix::ffi::OsStrExt;
    let path = std::ffi::CString::new( path.as_os_str().as_bytes() ).unwrap();
    let errcode = unsafe { libc::access( path.as_ptr(), libc::R_OK | libc::W_OK ) };

    errcode == 0
}

fn controller_main( device: nsinput::Device, slot: usize ) {
    // Do NOT change these unless you know what you're doing.
    //
    // These are only here mostly for debugging purposes.
    //
    // Yes, they *do* work just fine if you do change them,
    // however that might break SDL support as SDL's gamepad
    // presets are very picky when it comes to how a given
    // controller is exposed through evdev.

    let enable_digital_hat = true;
    let enable_analog_hat = false;
    let enable_digital_upper_shoulder = true;
    let enable_analog_upper_shoulder = false;
    let enable_digital_lower_shoulder = true;
    let enable_analog_lower_shoulder = false;

    let mut controller = match nsinput::Controller::open( &device ) {
        Ok( controller ) => controller,
        Err( error ) => {
            log::error!( "Failed to open the controller on {:03}:{:03}: {}", device.bus_number(), device.address(), error );
            match error {
                nsinput::Error::CannotOpenDevice( nsinput::UsbError::Access ) => {
                    log::error!( "You most likely can fix this by adding an appropriate udev rule to your system, e.g. by running:" );
                    log::error!( "   echo 'SUBSYSTEMS==\"usb\", ATTRS{{idVendor}}==\"057e\", ATTRS{{idProduct}}==\"2009\", MODE=\"0666\"' | sudo tee /etc/udev/rules.d/85-switch-pro-controller.rules" );
                    log::error!( "   sudo udevadm control --reload" );
                    log::error!( "And then you just have to reconnect your controller." );
                },
                _ => {}
            }
            return
        }
    };

    use nsinput::Led;
    match slot {
        1 => controller.set_led( Led::FIRST ),
        2 => controller.set_led( Led::SECOND ),
        3 => controller.set_led( Led::THIRD ),
        4 => controller.set_led( Led::FOURTH ),
        5 => controller.set_led( Led::FIRST | Led::SECOND ),
        6 => controller.set_led( Led::SECOND | Led::THIRD ),
        7 => controller.set_led( Led::THIRD | Led::FOURTH ),
        // Surely no sane person will go any higher than this...
        _ => controller.set_led( Led::FIRST | Led::SECOND | Led::THIRD | Led::FOURTH )
    }

    let device_id = DeviceId {
        // This is deliberately picked to be the same as "Nintendo Wii Remote Pro Controller"
        // since that is one of the very few controllers which are supported out-of-box by SDL while at
        // the same time having a sane layout which is compliant with the Linux Gamepad Specification.
        bus: Bus::Bluetooth,
        vendor: 0x057e,
        product: 0x0330,
        version: 1
    };

    let name = format!( "Switch Pro Controller #{}", slot );
    let mut event_bits: Vec< EventBit > = Vec::new();
    let keys = [
        Key::PadSouth,
        Key::PadEast,
        Key::PadNorth,
        Key::PadWest,
        Key::Select,
        Key::Start,
        Key::HomeButton,
        Key::StickLeft,
        Key::StickRight,

        // This is the "capture" button; we had to pick a keycode for this which is bigger than
        // all the rest so that we still match the SDL preset.
        Key::TriggerHappy
    ];

    for &key in keys.iter() {
        event_bits.push( EventBit::Key( key ) );
    }

    if enable_digital_hat {
        event_bits.push( EventBit::Key( Key::PadUp ) );
        event_bits.push( EventBit::Key( Key::PadDown ) );
        event_bits.push( EventBit::Key( Key::PadLeft ) );
        event_bits.push( EventBit::Key( Key::PadRight ) );
    }

    if enable_digital_upper_shoulder {
        event_bits.push( EventBit::Key( Key::ShoulderLeft ) );
        event_bits.push( EventBit::Key( Key::ShoulderRight ) );
    }

    if enable_digital_lower_shoulder {
        event_bits.push( EventBit::Key( Key::ShoulderLeftLower ) );
        event_bits.push( EventBit::Key( Key::ShoulderRightLower ) );
    }

    let axis_bit = AbsoluteAxisBit {
        axis: AbsoluteAxis::X,
        initial_value: 0,
        minimum: -65535,
        maximum: 65535,
        noise_threshold: 0,
        deadzone: 0,
        resolution: 0
    };

    event_bits.push( EventBit::AbsoluteAxis( AbsoluteAxisBit { axis: AbsoluteAxis::X, ..axis_bit } ) );
    event_bits.push( EventBit::AbsoluteAxis( AbsoluteAxisBit { axis: AbsoluteAxis::Y, ..axis_bit } ) );
    event_bits.push( EventBit::AbsoluteAxis( AbsoluteAxisBit { axis: AbsoluteAxis::RX, ..axis_bit } ) );
    event_bits.push( EventBit::AbsoluteAxis( AbsoluteAxisBit { axis: AbsoluteAxis::RY, ..axis_bit } ) );

    if enable_analog_hat {
        let axis_bit = AbsoluteAxisBit {
            axis: AbsoluteAxis::Hat0X,
            initial_value: 0,
            minimum: -1,
            maximum: 1,
            noise_threshold: 0,
            deadzone: 0,
            resolution: 0
        };

        event_bits.push( EventBit::AbsoluteAxis( AbsoluteAxisBit { axis: AbsoluteAxis::Hat0X, ..axis_bit } ) );
        event_bits.push( EventBit::AbsoluteAxis( AbsoluteAxisBit { axis: AbsoluteAxis::Hat0Y, ..axis_bit } ) );
    }

    if enable_analog_upper_shoulder {
        let axis_bit = AbsoluteAxisBit {
            axis: AbsoluteAxis::Hat1X,
            initial_value: 0,
            minimum: 0,
            maximum: 65535,
            noise_threshold: 0,
            deadzone: 0,
            resolution: 0
        };

        event_bits.push( EventBit::AbsoluteAxis( AbsoluteAxisBit { axis: AbsoluteAxis::Hat1X, ..axis_bit } ) );
        event_bits.push( EventBit::AbsoluteAxis( AbsoluteAxisBit { axis: AbsoluteAxis::Hat1Y, ..axis_bit } ) );
    }

    if enable_analog_lower_shoulder {
        let axis_bit = AbsoluteAxisBit {
            axis: AbsoluteAxis::Hat2X,
            initial_value: 0,
            minimum: 0,
            maximum: 65535,
            noise_threshold: 0,
            deadzone: 0,
            resolution: 0
        };
        event_bits.push( EventBit::AbsoluteAxis( AbsoluteAxisBit { axis: AbsoluteAxis::Hat2X, ..axis_bit } ) );
        event_bits.push( EventBit::AbsoluteAxis( AbsoluteAxisBit { axis: AbsoluteAxis::Hat2Y, ..axis_bit } ) );
    }

    event_bits.push( EventBit::ForceFeedback( ForceFeedback::Rumble ) );

    let virtual_device = linux_input::VirtualDevice::create( device_id, &name, event_bits ).expect( "failed to create a virtual device" );
    std::mem::drop( name );

    let virtual_device_path = virtual_device.path().expect( "failed to get the virtual device's path" );
    log::debug!( "Created a new virtual device at {:?}", virtual_device_path );

    // Wait until the device is set up by the system.
    std::thread::sleep( std::time::Duration::from_millis( 500 ) );

    if !is_accessible( &virtual_device_path ) {
        log::error!( "The virtual device which we've created for your controller is not accessible. You will NOT be able to use your controller." );
        log::error!( "You most likely can fix this by adding an appropriate udev rule to your system, e.g. by running:" );
        log::error!( "   echo 'SUBSYSTEMS==\"input\", ATTRS{{name}}==\"Switch Pro Controller #?\", MODE=\"0666\", ENV{{ID_INPUT_JOYSTICK}}=\"1\"' | sudo tee /etc/udev/rules.d/85-switch-pro-controller-virtual.rules" );
        log::error!( "   sudo udevadm control --reload" );
        log::error!( "And then you just have to reconnect your controller." );
    }

    // In case we're running as root - just make the device accessible to everyone.
    use std::os::unix::fs::PermissionsExt;
    let _ = std::fs::set_permissions( &virtual_device_path, std::fs::Permissions::from_mode( 0o666 ) );

    let mut last_buttons = controller.button_state();
    let mut last_left_stick = controller.left_stick();
    let mut last_right_stick = controller.right_stick();
    let mut last_hat_horizontal = 0;
    let mut last_hat_vertical = 0;
    let mut last_axis_upper_left = 0;
    let mut last_axis_upper_right = 0;
    let mut last_axis_lower_left = 0;
    let mut last_axis_lower_right = 0;

    let mut effect_0 = nsinput::RumbleData::disabled();
    let mut effect_0_delay = std::time::Duration::from_secs( 0 );
    let mut effect_0_duration = linux_input::ForceFeedbackDuration::Infinite;

    let mut effect_0_cycles_remaining = 0;
    let mut effect_0_running = false;
    let mut effect_0_start_at = std::time::Instant::now();
    let mut effect_0_end_at = None;

    while RUNNING.load( Ordering::SeqCst ) {
        if let Err( error ) = controller.poll_with_timeout( std::time::Duration::from_millis( 1 ) ) {
            log::warn!( "Encountered an error while polling controller #{}: {}", slot, error );
            return;
        }

        let now = std::time::Instant::now();

        loop {
            let request = virtual_device.poll_force_feedback( Some( std::time::Duration::from_secs( 0 ) ) )
                .expect( "failed to poll the virtual device for force_feedback" );

            match request {
                None => break,
                Some( ForceFeedbackRequest::Upload( request ) ) => {
                    let effect = request.effect();
                    log::debug!( "Force feedback effect #{} upload: {:?}", request.effect_id(), effect );

                    match effect.kind {
                        ForceFeedbackEffectKind::Rumble { weak_magnitude, strong_magnitude } => {
                            let weak_magnitude = weak_magnitude as f32 / std::u16::MAX as f32;
                            let strong_magnitude = strong_magnitude as f32 / std::u16::MAX as f32;

                            // TODO: This needs adjustment.
                            let low_channel_a = (weak_magnitude * 100.0) as u8;
                            let high_channel_a = (strong_magnitude * 100.0) as u8;

                            let rumble = nsinput::RumbleData::new_with_raw( 0x38, low_channel_a, 0x45, high_channel_a );
                            effect_0 = rumble;
                        }
                    }

                    effect_0_delay = effect.delay;
                    effect_0_duration = effect.duration;
                },
                Some( ForceFeedbackRequest::Erase( request ) ) => {
                    log::debug!( "Force feedback effect #{} erase", request.effect_id() );
                    effect_0 = nsinput::RumbleData::disabled();
                    effect_0_cycles_remaining = 0;
                },
                Some( ForceFeedbackRequest::Enable { effect_id, cycle_count } ) => {
                    log::debug!( "Force feedback effect #{} enabled (cycle_count = {})", effect_id, cycle_count );
                    effect_0_running = false;
                    effect_0_cycles_remaining = cycle_count;
                    effect_0_start_at = now + effect_0_delay;
                },
                Some( ForceFeedbackRequest::Disable { effect_id } ) => {
                    log::debug!( "Force feedback effect #{} disabled", effect_id );
                    effect_0_cycles_remaining = 0;
                },
                Some( ForceFeedbackRequest::Other { code, value } ) => {
                    log::debug!( "Unknown force feedback request: code={}, value={}", code, value );
                }
            }
        }

        loop {
            if effect_0_cycles_remaining > 0 {
                if !effect_0_running {
                    if effect_0_start_at <= now {
                        log::debug!( "Force feedback delay lapsed; enabling rumble..." );
                        effect_0_running = true;
                        effect_0_end_at = match effect_0_duration {
                            linux_input::ForceFeedbackDuration::Infinite => None,
                            linux_input::ForceFeedbackDuration::Finite( duration ) => Some( now + duration )
                        };
                    }
                } else {
                    if let Some( end_at ) = effect_0_end_at {
                        if now >= end_at {
                            effect_0_running = false;
                            effect_0_cycles_remaining -= 1;
                            effect_0_start_at = now + effect_0_delay;
                            if effect_0_cycles_remaining > 0 {
                                log::debug!( "Force feedback runtime lapsed; starting next cycle..." );
                            } else {
                                log::debug!( "Force feedback runtime lapsed; stopping..." );
                            }
                            continue;
                        }
                    }
                }
            } else {
                effect_0_running = false;
            }

            break;
        }

        if effect_0_running {
            controller.set_rumble( effect_0 );
        } else {
            controller.set_rumble( nsinput::RumbleData::disabled() );
        }

        let buttons = controller.button_state();

        let mut pending = false;
        for (button, is_pressed) in buttons.iter_changed( last_buttons ) {
            let key = match button {
                nsinput::Button::A => Key::PadEast,
                nsinput::Button::B => Key::PadSouth,
                nsinput::Button::Y => Key::PadWest,
                nsinput::Button::X => Key::PadNorth,
                nsinput::Button::LEFT if enable_digital_hat => Key::PadLeft,
                nsinput::Button::RIGHT if enable_digital_hat => Key::PadRight,
                nsinput::Button::UP if enable_digital_hat => Key::PadUp,
                nsinput::Button::DOWN if enable_digital_hat => Key::PadDown,
                nsinput::Button::L if enable_digital_upper_shoulder => Key::ShoulderLeft,
                nsinput::Button::R if enable_digital_upper_shoulder => Key::ShoulderRight,
                nsinput::Button::ZL if enable_digital_lower_shoulder => Key::ShoulderLeftLower,
                nsinput::Button::ZR if enable_digital_lower_shoulder => Key::ShoulderRightLower,
                nsinput::Button::LS => Key::StickLeft,
                nsinput::Button::RS => Key::StickRight,
                nsinput::Button::PLUS => Key::Start,
                nsinput::Button::MINUS => Key::Select,
                nsinput::Button::CAPTURE => Key::TriggerHappy,
                nsinput::Button::HOME => Key::HomeButton,
                _ => continue
            };

            let event = if is_pressed { InputEventBody::KeyPress( key ) } else { InputEventBody::KeyRelease( key ) };
            virtual_device.emit( event ).expect( "failed to send an event to the virtual device" );
            pending = true;
        }
        last_buttons = buttons;

        if enable_analog_upper_shoulder {
            let left = if buttons.is_pressed( nsinput::Button::L ) { 65535 } else { 0 };
            let right = if buttons.is_pressed( nsinput::Button::R ) { 65535 } else { 0 };

            if left != last_axis_upper_left {
                virtual_device.emit( InputEventBody::AbsoluteMove { axis: AbsoluteAxis::Hat1Y, position: left } )
                    .expect( "failed to send an event to the virtual device" );
                pending = true;
            }

            if right != last_axis_upper_right {
                virtual_device.emit( InputEventBody::AbsoluteMove { axis: AbsoluteAxis::Hat1X, position: right } )
                    .expect( "failed to send an event to the virtual device" );
                pending = true;
            }

            last_axis_upper_left = left;
            last_axis_upper_right = right;
        }

        if enable_analog_lower_shoulder {
            let left = if buttons.is_pressed( nsinput::Button::ZL ) { 65535 } else { 0 };
            let right = if buttons.is_pressed( nsinput::Button::ZR ) { 65535 } else { 0 };

            if left != last_axis_lower_left {
                virtual_device.emit( InputEventBody::AbsoluteMove { axis: AbsoluteAxis::Hat2Y, position: left } )
                    .expect( "failed to send an event to the virtual device" );
                pending = true;
            }

            if right != last_axis_lower_right {
                virtual_device.emit( InputEventBody::AbsoluteMove { axis: AbsoluteAxis::Hat2X, position: right } )
                    .expect( "failed to send an event to the virtual device" );
                pending = true;
            }

            last_axis_lower_left = left;
            last_axis_lower_right = right;
        }

        if enable_analog_hat {
            let hat_horizontal = if buttons.is_pressed( nsinput::Button::LEFT ) {
                -1
            } else if buttons.is_pressed( nsinput::Button::RIGHT ) {
                1
            } else {
                0
            };

            let hat_vertical = if buttons.is_pressed( nsinput::Button::UP ) {
                -1
            } else if buttons.is_pressed( nsinput::Button::DOWN ) {
                1
            } else {
                0
            };

            if last_hat_horizontal != hat_horizontal {
                virtual_device.emit( InputEventBody::AbsoluteMove { axis: AbsoluteAxis::Hat0X, position: hat_horizontal } )
                    .expect( "failed to send an event to the virtual device" );
                pending = true;
                last_hat_horizontal = hat_horizontal;
            }

            if last_hat_vertical != hat_vertical {
                virtual_device.emit( InputEventBody::AbsoluteMove { axis: AbsoluteAxis::Hat0Y, position: hat_vertical } )
                    .expect( "failed to send an event to the virtual device" );
                pending = true;
                last_hat_vertical = hat_vertical;
            }
        }

        let left_stick = controller.left_stick();
        let right_stick = controller.right_stick();

        if left_stick.x != last_left_stick.x {
            virtual_device.emit( InputEventBody::AbsoluteMove { axis: AbsoluteAxis::X, position: (left_stick.x * 65535.0) as i32 } )
                .expect( "failed to send an event to the virtual device" );
            pending = true;
        }
        if left_stick.y != last_left_stick.y {
            virtual_device.emit( InputEventBody::AbsoluteMove { axis: AbsoluteAxis::Y, position: (left_stick.y * 65535.0) as i32 } )
                .expect( "failed to send an event to the virtual device" );
            pending = true;
        }
        if right_stick.x != last_right_stick.x {
            virtual_device.emit( InputEventBody::AbsoluteMove { axis: AbsoluteAxis::RX, position: (right_stick.x * 65535.0) as i32 } )
                .expect( "failed to send an event to the virtual device" );
            pending = true;
        }
        if right_stick.y != last_right_stick.y {
            virtual_device.emit( InputEventBody::AbsoluteMove { axis: AbsoluteAxis::RY, position: (right_stick.y * 65535.0) as i32 } )
                .expect( "failed to send an event to the virtual device" );
            pending = true;
        }

        last_left_stick = left_stick;
        last_right_stick = right_stick;

        if pending {
            let event = InputEvent {
                timestamp: Default::default(),
                body: InputEventBody::Flush
            };
            virtual_device.emit( event ).expect( "failed to flush pending virtual device events" );
        }
    }
}

fn test_main_1( device: nsinput::Device ) {
    let mut controller = nsinput::Controller::open( &device ).unwrap();

    let a = nsinput::RumbleData::new_with_raw( 0x40, 70, 0, 0 );
    let b = nsinput::RumbleData::new_with_raw( 0, 0, 0, 0 );
    let mut timestamp = std::time::Instant::now();
    let mut n = 0;
    while RUNNING.load( Ordering::SeqCst ) {
        if timestamp.elapsed() >= std::time::Duration::from_millis( 62 ) {
            timestamp = std::time::Instant::now();
            if (n % 2) == 0 {
                controller.set_rumble( a );
                controller.set_led( nsinput::Led::FIRST );
            } else {
                controller.set_rumble( b );
                controller.set_led( nsinput::Led::SECOND );
            }
            n += 1;
        }
        controller.poll_with_timeout( std::time::Duration::from_millis( 1 ) ).unwrap();
    }
}

fn test_main_2( device: nsinput::Device ) {
    let mut controller = nsinput::Controller::open( &device ).unwrap();
    let mut last_buttons = controller.button_state();
    let mut low_channel_f = 0x40;
    let mut low_channel_a = 0;
    let mut high_channel_f = 0x40;
    let mut high_channel_a = 0;
    let mut refresh = true;
    let mut selected = 0;
    let mut delta = None;
    let mut last_change = std::time::Instant::now();
    let mut wait_duration = std::time::Duration::from_secs( 0 );
    while RUNNING.load( Ordering::SeqCst ) {
        controller.poll_with_timeout( std::time::Duration::from_millis( 1 ) ).unwrap();

        let buttons = controller.button_state();
        for (button, is_pressed) in buttons.iter_changed( last_buttons ) {
            if !is_pressed {
                continue;
            }
            match button {
                nsinput::Button::LEFT if is_pressed && selected > 0 => selected -= 1,
                nsinput::Button::RIGHT if is_pressed && selected < 3 => selected += 1,
                nsinput::Button::UP | nsinput::Button::DOWN => {
                    if is_pressed {
                        delta = Some( if button == nsinput::Button::UP { 1 } else { -1 } );
                        last_change = std::time::Instant::now();
                        wait_duration = std::time::Duration::from_secs( 0 );
                    }
                },
                _ => {}
            }
            refresh = true;
        }
        last_buttons = buttons;

        if !buttons.is_pressed( nsinput::Button::UP ) && !buttons.is_pressed( nsinput::Button::DOWN ) {
            delta = None;
        }

        if let Some( mut delta ) = delta {
            if buttons.is_pressed( nsinput::Button::ZR ) {
                delta *= 16;
            }

            let (target, min, max) =
                if selected == 0 {
                    (&mut low_channel_f, 1, 127)
                } else if selected == 1 {
                    (&mut low_channel_a, 0, 100)
                } else if selected == 2 {
                    (&mut high_channel_f, 1, 127)
                } else if selected == 3 {
                    (&mut high_channel_a, 0, 100)
                } else {
                    unreachable!()
                };

            let deadline = last_change + wait_duration;
            let now = std::time::Instant::now();
            if now >= deadline {
                let mut new_value = *target as i32 + delta;
                if new_value < min as i32 {
                    new_value = min as i32;
                }
                if new_value > max as i32 {
                    new_value = max as i32;
                }
                *target = new_value as u8;
                wait_duration = std::time::Duration::from_millis( 100 );
                last_change = last_change + wait_duration;
                refresh = true;
            }
        }

        if refresh {
            refresh = false;
            let rumble = nsinput::RumbleData::new_with_raw( low_channel_f, low_channel_a, high_channel_f, high_channel_a );
            controller.set_rumble( rumble );

            let color = "\x1B[1;30;47m";
            let color_lf = if selected == 0 { color } else { "" };
            let color_la = if selected == 1 { color } else { "" };
            let color_hf = if selected == 2 { color } else { "" };
            let color_ha = if selected == 3 { color } else { "" };

            print!(
                "\x1B[2K\rLF: {}{:02X}\x1B[0m    LA: {}{:02X}\x1B[0m    HF: {}{:02X}\x1B[0m    HA: {}{:02X}\x1B[0m",
                color_lf, low_channel_f, color_la, low_channel_a,
                color_hf, high_channel_f, color_ha, high_channel_a
            );
            use std::io::Write;
            if std::io::stdout().lock().flush().is_err() {
                break;
            }
        }
    }

    println!();
}

fn main() -> Result< (), Box< dyn std::error::Error > > {
    if std::env::var( "RUST_LOG" ).is_err() {
        std::env::set_var( "RUST_LOG", "info" );
    }

    let test_mode: u32 = std::env::var( "ENABLE_TEST_MODE" ).map( |value| value.parse().unwrap() ).unwrap_or( 0 );

    env_logger::init();

    let usb = nsinput::UsbContext::new()?;
    let hotplug = nsinput::hotplug_events( &usb )?;

    ctrlc::set_handler( move || {
        RUNNING.store( false, Ordering::SeqCst );
    }).unwrap();

    let mut used_slots = HashSet::new();
    let mut address_to_slot = HashMap::new();

    log::info!( "Waiting for controllers..." );
    while RUNNING.load( Ordering::SeqCst ) {
        let event = match hotplug.recv_timeout( std::time::Duration::from_millis( 100 ) ) {
            Some( event ) => event,
            None => continue
        };

        match event {
            nsinput::HotplugEvent::Connected( device ) => {
                let mut slot = 1;
                loop {
                    if !used_slots.contains( &slot ) {
                        used_slots.insert( slot );
                        address_to_slot.insert( (device.bus_number(), device.address()), slot );
                        break;
                    }
                    slot += 1;
                }

                log::info!( "Detected a new controller on {:03}:{:03}; designating as controller #{}", device.bus_number(), device.address(), slot );
                std::thread::spawn( move || {
                    if test_mode == 1 {
                        test_main_1( device )
                    } else if test_mode == 2 {
                        test_main_2( device )
                    } else {
                        controller_main( device, slot )
                    }
                });
            },
            nsinput::HotplugEvent::Disconnected( device ) => {
                let slot = address_to_slot.remove( &(device.bus_number(), device.address()) ).unwrap();
                used_slots.remove( &slot );
                log::info!( "Controller #{} on {:03}:{:03} was disconnected", slot, device.bus_number(), device.address() );
            }
        }
    }

    Ok(())
}
