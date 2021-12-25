use {
    lock_api::{
        RawMutex as RawMutexT,
        RawMutexTimed as RawMutexTimedT
    },
    parking_lot::{
        Mutex
    },
    std::{
        cell::{
            UnsafeCell
        },
        collections::{
            HashSet
        },
        convert::{
            TryInto
        },
        mem,
        ptr::{
            NonNull
        },
        sync::{
            Arc,
            atomic::{
                AtomicBool,
                Ordering
            }
        },
        time::{
            Duration
        }
    }
};

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Error {
    Io,
    InvalidParam,
    Access,
    NoDevice,
    NotFound,
    Busy,
    Timeout,
    Overflow,
    Pipe,
    Interrupted,
    NoMem,
    NotSupported,
    Other,
}

impl std::error::Error for Error {}

impl std::fmt::Display for Error {
    fn fmt( &self, fmt: &mut std::fmt::Formatter ) -> std::fmt::Result {
        match *self {
            Error::Io => write!( fmt, "IO error" ),
            Error::InvalidParam => write!( fmt, "invalid parameter" ),
            Error::Access => write!( fmt, "access denied" ),
            Error::NoDevice => write!( fmt, "no device found" ),
            Error::NotFound => write!( fmt, "entity not found" ),
            Error::Busy => write!( fmt, "resource busy" ),
            Error::Timeout => write!( fmt, "operation timed-out" ),
            Error::Overflow => write!( fmt, "overflow" ),
            Error::Pipe => write!( fmt, "pipe error" ),
            Error::Interrupted => write!( fmt, "interrupted" ),
            Error::NoMem => write!( fmt, "out of memory" ),
            Error::NotSupported => write!( fmt, "not supported" ),
            Error::Other => write!( fmt, "other" )
        }
    }
}

fn into_result( error: i32 ) -> Result< u32, Error > {
    use libusb1_sys::constants::*;
    let error = match error {
        error if error >= 0 => return Ok( error as u32 ),
        LIBUSB_ERROR_IO => Error::Io,
        LIBUSB_ERROR_INVALID_PARAM => Error::InvalidParam,
        LIBUSB_ERROR_ACCESS => Error::Access,
        LIBUSB_ERROR_NO_DEVICE => Error::NoDevice,
        LIBUSB_ERROR_NOT_FOUND => Error::NotFound,
        LIBUSB_ERROR_BUSY => Error::Busy,
        LIBUSB_ERROR_TIMEOUT => Error::Timeout,
        LIBUSB_ERROR_OVERFLOW => Error::Overflow,
        LIBUSB_ERROR_PIPE => Error::Pipe,
        LIBUSB_ERROR_INTERRUPTED => Error::Interrupted,
        LIBUSB_ERROR_NO_MEM => Error::NoMem,
        LIBUSB_ERROR_NOT_SUPPORTED => Error::NotSupported,
        LIBUSB_ERROR_OTHER | _ => Error::Other,
    };

    Err( error )
}

struct ContextObject {
    raw_context: NonNull< libusb1_sys::libusb_context >,
    running: Arc< AtomicBool >,
    thread: Option< std::thread::JoinHandle< () > >,
}

unsafe impl Send for ContextObject {}
unsafe impl Sync for ContextObject {}

impl Drop for ContextObject {
    fn drop( &mut self ) {
        self.running.store( false, Ordering::SeqCst );
        let _ = self.thread.take().unwrap().join();
        unsafe {
            libusb1_sys::libusb_exit( self.raw_context.as_ptr() );
        }
    }
}

pub enum HotplugEvent {
    Connected( Device ),
    Disconnected( Device )
}

struct HotplugHandler {
    context: Context,
    seen: HashSet< (u8, u8) >,
    handle: Option< (libusb1_sys::libusb_hotplug_callback_handle, *const Mutex< HotplugHandler >) >,
    tx: flume::Sender< HotplugEvent >
}

pub struct HotplugStream {
    handler: Arc< Mutex< HotplugHandler > >,
    rx: flume::Receiver< HotplugEvent >
}

impl HotplugStream {
    pub fn recv( &self ) -> HotplugEvent {
        self.rx.recv().unwrap()
    }

    pub fn recv_timeout( &self, timeout: Duration ) -> Option< HotplugEvent > {
        match self.rx.recv_timeout( timeout ) {
            Ok( event ) => Some( event ),
            Err( flume::RecvTimeoutError::Timeout ) => None,
            Err( flume::RecvTimeoutError::Disconnected ) => unreachable!()
        }
    }
}

// Unfortunately libusb's docs aren't very detailed when it comes to
// describing how `libusb_hotplug_deregister_callback` behaves when
// we have a concurrent hotplug callback in-progress. Does it wait
// for the callback to finish, or not? Is it atomic? No idea.
//
// So I'm not sure if this is actually 100% safe.
impl Drop for HotplugStream {
    fn drop( &mut self ) {
        let handler = self.handler.lock();
        let (handle, pointer) = handler.handle.unwrap();
        unsafe {
            libusb1_sys::libusb_hotplug_deregister_callback(
                handler.context.0.raw_context.as_ptr(),
                handle
            );
            std::mem::drop( Arc::from_raw( pointer ) );
        }
        std::mem::drop( handler );
    }
}

#[derive(Clone)]
pub struct Context( Arc< ContextObject > );

impl Context {
    pub fn new() -> Result< Self, Error > {
        let mut raw_context = std::mem::MaybeUninit::uninit();
        into_result( unsafe {
            libusb1_sys::libusb_init( raw_context.as_mut_ptr() )
        })?;

        let raw_context = NonNull::new( unsafe { raw_context.assume_init() } ).unwrap();
        let running = Arc::new( AtomicBool::new( true ) );
        let thread = {
            struct RawContext( NonNull< libusb1_sys::libusb_context > );
            unsafe impl Send for RawContext {}
            unsafe impl Sync for RawContext {}

            let raw_context = RawContext( raw_context );
            let running = running.clone();
            std::thread::Builder::new().name( "libusb".into() ).spawn( move || {
                while running.load( Ordering::Relaxed ) {
                    let tv = libc::timeval {
                        tv_sec: 0,
                        tv_usec: 100000
                    };
                    into_result( unsafe {
                        libusb1_sys::libusb_handle_events_timeout( raw_context.0.as_ptr(), &tv )
                    }).unwrap();
                }
            }).expect( "failed to launch a thread" )
        };

        let context = Context( Arc::new( ContextObject {
            raw_context,
            running,
            thread: Some( thread ),
        }));

        Ok( context )
    }

    pub fn devices( &self ) -> Result< Vec< Device >, Error > {
        let mut list = std::mem::MaybeUninit::uninit();
        let count = into_result( unsafe {
            libusb1_sys::libusb_get_device_list(
                self.0.raw_context.as_ptr(),
                list.as_mut_ptr()
            )
        } as _ )?;

        let list = unsafe { list.assume_init() };
        let slice: &[*mut libusb1_sys::libusb_device] = unsafe { std::slice::from_raw_parts( list, count as usize ) };
        let mut devices = Vec::new();

        for &raw_device in slice {
            match unsafe { Device::from_raw( self, raw_device, true ) } {
                Ok( device ) => devices.push( device ),
                Err( _ ) => unsafe { libusb1_sys::libusb_unref_device( raw_device ) }
            }
        }

        unsafe {
            libusb1_sys::libusb_free_device_list( list, 0 );
        }

        Ok( devices )
    }

    pub fn hotplug_events(
        &self,
        vendor_id: Option< u16 >,
        product_id: Option< u16 >,
        device_class: Option< u8 >
    ) -> Result< HotplugStream, Error > {
        let (tx, rx) = flume::unbounded();
        let handler = Arc::new( Mutex::new( HotplugHandler {
            context: self.clone(),
            seen: HashSet::new(),
            handle: None,
            tx
        }));

        extern "system" fn callback(
            _: *mut libusb1_sys::libusb_context,
            device: *mut libusb1_sys::libusb_device,
            event: libusb1_sys::libusb_hotplug_event,
            user_data: *mut std::ffi::c_void
        ) -> i32 {
            let handler_pointer = user_data as *const Mutex< HotplugHandler >;
            let handler = unsafe { &*handler_pointer };
            let mut handler = handler.lock();

            let device = match unsafe { Device::from_raw( &handler.context, device, false ) } {
                Ok( device ) => device,
                Err( _ ) => return 0
            };

            let event = match event {
                libusb1_sys::constants::LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED => {
                    let id = (device.bus_number(), device.address());
                    debug!( "Hotplug device arrived: {:03}:{:03}", id.0, id.1 );

                    if handler.seen.contains( &id ) {
                        return 0;
                    }
                    handler.seen.insert( id );
                    HotplugEvent::Connected( device )
                },
                libusb1_sys::constants::LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT => {
                    let id = (device.bus_number(), device.address());
                    debug!( "Hotplug device left: {:03}:{:03}", id.0, id.1 );

                    if !handler.seen.remove( &id ) {
                        return 0;
                    }
                    HotplugEvent::Disconnected( device )
                },
                _ => unreachable!()
            };

            let _ = handler.tx.send( event );
            0
        }

        let mut handle = 0;
        let handler_pointer = Arc::into_raw( handler.clone() );
        let result = unsafe {
            libusb1_sys::libusb_hotplug_register_callback(
                self.0.raw_context.as_ptr(),
                libusb1_sys::constants::LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | libusb1_sys::constants::LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
                libusb1_sys::constants::LIBUSB_HOTPLUG_ENUMERATE,
                vendor_id.map( |value| value as i32 ).unwrap_or( libusb1_sys::constants::LIBUSB_HOTPLUG_MATCH_ANY ),
                product_id.map( |value| value as i32 ).unwrap_or( libusb1_sys::constants::LIBUSB_HOTPLUG_MATCH_ANY ),
                device_class.map( |value| value as i32 ).unwrap_or( libusb1_sys::constants::LIBUSB_HOTPLUG_MATCH_ANY ),
                callback,
                handler_pointer as *mut std::ffi::c_void,
                &mut handle
            )
        };

        if let Err( error ) = into_result( result ) {
            std::mem::drop( unsafe { Arc::from_raw( handler_pointer ) } );
            return Err( error );
        }

        {
            let mut handler = handler.lock();
            handler.handle = Some( (handle, handler_pointer) );
        }

        Ok( HotplugStream {
            handler,
            rx
        })
    }
}

pub struct Device {
    context: Context,
    raw_device: NonNull< libusb1_sys::libusb_device >,
    vendor_id: u16,
    product_id: u16,
    bus_number: u8,
    address: u8
}

unsafe impl Send for Device {}

impl Clone for Device {
    fn clone( &self ) -> Self {
        unsafe {
            libusb1_sys::libusb_ref_device( self.raw_device.as_ptr() );
        }
        Device {
            context: self.context.clone(),
            raw_device: self.raw_device,
            vendor_id: self.vendor_id,
            product_id: self.product_id,
            bus_number: self.bus_number,
            address: self.address
        }
    }
}

impl Drop for Device {
    fn drop( &mut self ) {
        unsafe {
            libusb1_sys::libusb_unref_device( self.raw_device.as_ptr() )
        }
    }
}

struct DeviceHandleObject {
    raw_handle: NonNull< libusb1_sys::libusb_device_handle >
}

unsafe impl Send for DeviceHandleObject {}

impl Drop for DeviceHandleObject {
    fn drop( &mut self ) {
        unsafe {
            libusb1_sys::libusb_close( self.raw_handle.as_ptr() );
        }
    }
}

pub struct DeviceHandle( DeviceHandleObject );

impl Device {
    unsafe fn from_raw( context: &Context, raw_device: *mut libusb1_sys::libusb_device, is_owned: bool ) -> Result< Self, Error > {
        let raw_device = NonNull::new( raw_device ).unwrap();
        let mut descriptor = std::mem::MaybeUninit::uninit();
        into_result( libusb1_sys::libusb_get_device_descriptor( raw_device.as_ptr(), descriptor.as_mut_ptr() ) )?;

        if !is_owned {
            libusb1_sys::libusb_ref_device( raw_device.as_ptr() );
        }

        let descriptor = descriptor.assume_init();
        let device = Device {
            context: context.clone(),
            vendor_id: descriptor.idVendor,
            product_id: descriptor.idProduct,
            bus_number: libusb1_sys::libusb_get_bus_number( raw_device.as_ptr() ),
            address: libusb1_sys::libusb_get_device_address( raw_device.as_ptr() ),
            raw_device: raw_device
        };

        Ok( device )
    }

    pub fn open( &self ) -> Result< DeviceHandle, Error > {
        DeviceHandle::open( self )
    }

    pub fn vendor_id( &self ) -> u16 {
        self.vendor_id
    }

    pub fn product_id( &self ) -> u16 {
        self.product_id
    }

    pub fn bus_number( &self ) -> u8 {
        self.bus_number
    }

    pub fn address( &self ) -> u8 {
        self.address
    }
}

struct TransferBody {
    raw_handle: *mut libusb1_sys::libusb_transfer,
    buffer: Vec< u8 >,
    error: Option< TransferError >,
    transferred: usize,
    sender: Option< flume::Sender< TransferLock > >
}

impl Drop for TransferBody {
    fn drop( &mut self ) {
        unsafe {
            libusb1_sys::libusb_free_transfer( self.raw_handle );
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum TransferError {
    Error,
    TimedOut,
    Cancelled,
    Stall,
    NoDevice,
    Overflow,
    Other( i32 )
}

impl From< TransferError > for Error {
    fn from( error: TransferError ) -> Error {
        match error {
            TransferError::Error => Error::Other,
            TransferError::TimedOut => Error::Timeout,
            TransferError::Cancelled => Error::Interrupted,
            TransferError::Stall => Error::Busy,
            TransferError::NoDevice => Error::NoDevice,
            TransferError::Overflow => Error::Overflow,
            TransferError::Other( _ ) => Error::Other
        }
    }
}

impl std::fmt::Display for TransferError {
    fn fmt( &self, fmt: &mut std::fmt::Formatter ) -> std::fmt::Result {
        match *self {
            TransferError::Error => write!( fmt, "transfer failed" ),
            TransferError::TimedOut => write!( fmt, "transfer timed-out" ),
            TransferError::Cancelled => write!( fmt, "transfer was cancelled" ),
            TransferError::Stall => write!( fmt, "transfer stalled" ),
            TransferError::NoDevice => write!( fmt, "no device found" ),
            TransferError::Overflow => write!( fmt, "transfer overflow" ),
            TransferError::Other( error ) => write!( fmt, "error {}", error )
        }
    }
}

struct TransferObject {
    mutex: parking_lot::RawMutex,
    body: UnsafeCell< TransferBody >
}

pub struct TransferLock( Arc< TransferObject > );

impl Drop for TransferLock {
    fn drop( &mut self ) {
        self.0.mutex.unlock();
    }
}

impl TransferLock {
    fn body( &self ) -> &TransferBody {
        unsafe { &*self.0.body.get() }
    }

    fn body_mut( &mut self ) -> &mut TransferBody {
        unsafe { &mut *self.0.body.get() }
    }

    pub fn unlock( self ) -> Transfer {
        let transfer = Transfer( self.0.clone() );
        std::mem::drop( self );
        transfer
    }

    pub fn set_data( &mut self, data: &[u8] ) {
        let body = self.body_mut();
        body.buffer.clear();
        body.buffer.extend_from_slice( data );
    }

    pub fn buffer( &self ) -> &[u8] {
        &self.body().buffer
    }

    pub fn buffer_mut( &mut self ) -> &mut Vec< u8 > {
        &mut self.body_mut().buffer
    }

    pub fn error( &self ) -> Option< TransferError > {
        self.body().error
    }

    pub fn bytes_transferred( &self ) -> usize {
        self.body().transferred
    }

    pub fn set_completion_sender( &mut self, sender: flume::Sender< TransferLock > ) {
        self.body_mut().sender = Some( sender );
    }

    pub fn submit_interrupt( mut self, device_handle: &DeviceHandle, endpoint: u8, timeout: Duration ) -> Result< (), Error > {
        let body = self.body_mut();
        let raw_handle = unsafe { &mut *body.raw_handle };
        raw_handle.dev_handle = device_handle.0.raw_handle.as_ptr();
        raw_handle.endpoint = endpoint;
        raw_handle.transfer_type = libusb1_sys::constants::LIBUSB_TRANSFER_TYPE_INTERRUPT;
        raw_handle.timeout = timeout.as_millis() as _;
        raw_handle.buffer = body.buffer.as_mut_ptr();
        raw_handle.length = body.buffer.len().try_into().unwrap();
        raw_handle.user_data = Arc::into_raw( self.0.clone() ) as *mut std::ffi::c_void;
        raw_handle.flags = libusb1_sys::constants::LIBUSB_TRANSFER_SHORT_NOT_OK;
        extern "system" fn callback( raw_handle: *mut libusb1_sys::libusb_transfer ) {
            unsafe {
                let pointer = (*raw_handle).user_data as _;
                let mut transfer = TransferLock( Arc::from_raw( pointer ) );
                let status = (*raw_handle).status;
                let transferred = (*raw_handle).actual_length;
                let error = match status {
                    libusb1_sys::constants::LIBUSB_TRANSFER_COMPLETED => None,
                    libusb1_sys::constants::LIBUSB_TRANSFER_ERROR => Some( TransferError::Error ),
                    libusb1_sys::constants::LIBUSB_TRANSFER_TIMED_OUT => Some( TransferError::TimedOut ),
                    libusb1_sys::constants::LIBUSB_TRANSFER_CANCELLED => Some( TransferError::Cancelled ),
                    libusb1_sys::constants::LIBUSB_TRANSFER_STALL => Some( TransferError::Stall ),
                    libusb1_sys::constants::LIBUSB_TRANSFER_NO_DEVICE => Some( TransferError::NoDevice ),
                    libusb1_sys::constants::LIBUSB_TRANSFER_OVERFLOW => Some( TransferError::Overflow ),
                    status => Some( TransferError::Other( status ) )
                };

                let body = transfer.body_mut();
                body.error = error;
                body.transferred = std::cmp::max( transferred, 0 ) as usize;

                if let Some( ref sender ) = body.sender {
                    let sender = sender.clone();
                    let _ = sender.send( transfer );
                }
            }
        }
        raw_handle.callback = callback;
        mem::forget( self ); // Leak the guard; we'll unlock it in the callback.

        let errcode = unsafe { libusb1_sys::libusb_submit_transfer( raw_handle ) };
        if let Err( error ) = into_result( errcode ) {
            unsafe {
                let pointer = (*raw_handle).user_data as _;
                let mut transfer = TransferLock( Arc::from_raw( pointer ) );

                let body = transfer.body_mut();
                body.error = Some( TransferError::Error );
                body.transferred = 0;
            }
            return Err( error );
        }

        Ok(())
    }
}

#[derive(Clone)]
pub struct Transfer( Arc< TransferObject > );

impl PartialEq< TransferLock > for Transfer {
    fn eq( &self, rhs: &TransferLock ) -> bool {
        Arc::ptr_eq( &self.0, &rhs.0 )
    }
}

impl PartialEq< Transfer > for TransferLock {
    fn eq( &self, rhs: &Transfer ) -> bool {
        Arc::ptr_eq( &self.0, &rhs.0 )
    }
}

impl Transfer {
    pub fn new() -> Self {
        Transfer( Arc::new( TransferObject {
            mutex: parking_lot::RawMutex::INIT,
            body: UnsafeCell::new( TransferBody {
                raw_handle: unsafe { libusb1_sys::libusb_alloc_transfer( 0 ) },
                buffer: Vec::new(),
                error: None,
                transferred: 0,
                sender: None
            })
        }))
    }

    pub fn lock( &self ) -> TransferLock {
        let transfer = self.0.clone();

        self.0.mutex.lock();
        TransferLock( transfer )
    }

    pub fn try_lock( &self ) -> Option< TransferLock > {
        let transfer = self.0.clone();

        if self.0.mutex.try_lock() {
            Some( TransferLock( transfer ) )
        } else {
            None
        }
    }

    #[allow(dead_code)]
    pub fn try_lock_with_timeout( &self, timeout: Duration ) -> Option< TransferLock > {
        let transfer = self.0.clone();

        if self.0.mutex.try_lock_for( timeout ) {
            Some( TransferLock( transfer ) )
        } else {
            None
        }
    }
}

impl DeviceHandle {
    pub fn open( device: &Device ) -> Result< Self, Error > {
        let mut raw_handle = std::mem::MaybeUninit::uninit();
        into_result( unsafe {
            libusb1_sys::libusb_open(
                device.raw_device.as_ptr(),
                raw_handle.as_mut_ptr()
            )
        })?;

        let raw_handle = NonNull::new( unsafe { raw_handle.assume_init() } ).unwrap();
        let handle = DeviceHandle( DeviceHandleObject {
            raw_handle
        });

        Ok( handle )
    }

    pub fn read_interrupt(
        &self,
        endpoint: u8,
        buffer: &mut [u8],
        timeout: Duration
    ) -> Result< usize, Error > {
        if endpoint & libusb1_sys::constants::LIBUSB_ENDPOINT_DIR_MASK != libusb1_sys::constants::LIBUSB_ENDPOINT_IN {
            return Err( Error::InvalidParam );
        }

        let transfer = Transfer::new();
        {
            let mut transfer = transfer.lock();
            transfer.buffer_mut().resize( buffer.len(), 0 );
            transfer.submit_interrupt( self, endpoint, timeout )?;
        }

        let transfer = transfer.lock();
        match transfer.error() {
            None => {
                let length = transfer.bytes_transferred();
                assert_eq!( length, buffer.len() );

                buffer.copy_from_slice( &transfer.buffer()[ 0..length ] );
                Ok( length )
            },
            Some( error ) => {
                Err( error.into() )
            }
        }
    }

    pub fn write_interrupt(
        &self,
        endpoint: u8,
        buffer: &[u8],
        timeout: Duration
    ) -> Result< usize, Error > {
        if endpoint & libusb1_sys::constants::LIBUSB_ENDPOINT_DIR_MASK != libusb1_sys::constants::LIBUSB_ENDPOINT_OUT {
            return Err( Error::InvalidParam );
        }

        let transfer = Transfer::new();
        {
            let mut transfer = transfer.lock();
            transfer.set_data( buffer );
            transfer.submit_interrupt( self, endpoint, timeout )?;
        }

        let transfer = transfer.lock();
        match transfer.error() {
            None => {
                let length = transfer.bytes_transferred();
                assert_eq!( length, buffer.len() );

                Ok( length )
            },
            Some( error ) => {
                Err( error.into() )
            }
        }
    }

    pub fn kernel_driver_active( &self, interface: u8 ) -> Result< bool, Error > {
        let result = into_result( unsafe {
            libusb1_sys::libusb_kernel_driver_active( self.0.raw_handle.as_ptr(), interface as _ )
        })?;
        Ok( result != 0 )
    }

    pub fn detach_kernel_driver( &mut self, interface: u8 ) -> Result< (), Error > {
        into_result( unsafe {
            libusb1_sys::libusb_detach_kernel_driver( self.0.raw_handle.as_ptr(), interface as _ )
        })?;
        Ok(())
    }

    pub fn active_configuration( &self ) -> Result< u8, Error > {
        let mut config = std::mem::MaybeUninit::uninit();
        into_result( unsafe {
            libusb1_sys::libusb_get_configuration( self.0.raw_handle.as_ptr(), config.as_mut_ptr() )
        })?;
        Ok( unsafe { config.assume_init() } as u8 )
    }

    pub fn set_active_configuration( &mut self, config: u8 ) -> Result< (), Error > {
        into_result( unsafe {
            libusb1_sys::libusb_set_configuration( self.0.raw_handle.as_ptr(), config as _ )
        })?;
        Ok(())
    }

    pub fn claim_interface( &mut self, interface: u8 ) -> Result< (), Error > {
        into_result( unsafe {
            libusb1_sys::libusb_claim_interface( self.0.raw_handle.as_ptr(), interface as _ )
        })?;
        Ok(())
    }
}
