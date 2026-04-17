//! Convenience types for mounting a FAT filesystem on an SD card via SPI.

use aligned::A1;
use block_device_adapters::{BufStream, BufStreamError};
use embassy_sync::blocking_mutex::raw::RawMutex;
use sdspi::SdSpi;

use crate::{FileSystem, FsOptions, OemCpConverter, TimeProvider};

/// The full I/O stack for an SD card: `SdSpi` wrapped in `BufStream`.
pub type SdBlockDevice<SPI, CS, D> = BufStream<SdSpi<SPI, CS, D, A1>, 512>;

/// A FAT filesystem mounted on top of an SD card SPI driver.
pub type SdFatFs<SPI, CS, D, TP, OCC, M> = FileSystem<SdBlockDevice<SPI, CS, D>, TP, OCC, M>;

/// Errors that can occur while mounting an SD card filesystem.
#[derive(Debug)]
pub enum SdMountError<StorageError: core::fmt::Debug> {
    /// SD card hardware initialisation failed.
    SdInit(sdspi::Error),
    /// FAT filesystem mount failed after the card was initialised.
    FsMount(crate::Error<BufStreamError<StorageError>>),
}

/// Initialise an SD card over SPI and mount a FAT filesystem on it.
///
/// This combines the boilerplate steps that every caller repeats:
/// 1. Pre-clock the card (`sd_init`)
/// 2. Initialise the `SdSpi` driver
/// 3. Wrap it in `BufStream` and call `FileSystem::new`
///
/// # Errors
/// Returns [`SdMountError::SdInit`] if the SD card fails to initialise, or
/// [`SdMountError::FsMount`] if the FAT structures on the card are invalid.
pub async fn mount_sd_spi<SPI, CS, D, TP, OCC, M>(
    mut spi: SPI,
    mut cs: CS,
    delay: D,
    options: FsOptions<TP, OCC>,
) -> Result<SdFatFs<SPI, CS, D, TP, OCC, M>, SdMountError<sdspi::Error>>
where
    SPI: embedded_hal_async::spi::SpiBus,
    CS: embedded_hal::digital::OutputPin,
    D: embedded_hal_async::delay::DelayNs + Clone,
    TP: TimeProvider,
    OCC: OemCpConverter,
    M: RawMutex,
{
    sdspi::sd_init(&mut spi, &mut cs)
        .await
        .map_err(SdMountError::SdInit)?;

    let mut sd = SdSpi::<_, _, _, A1>::new(spi, cs, delay);
    sd.init().await.map_err(SdMountError::SdInit)?;

    let io = BufStream::<_, 512>::new(sd);
    FileSystem::new(io, options).await.map_err(SdMountError::FsMount)
}
