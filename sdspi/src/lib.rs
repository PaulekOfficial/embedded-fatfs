//! A crate for interfacing with SD cards over SPI.

#![no_std]

use aligned::Aligned;
use core::future::Future;
use core::marker::PhantomData;
use embassy_futures::select::{select, Either};
use sdio_host::sd::{CardCapacity, CID, CSD, OCR, SD};
use sdio_host::{common_cmd::*, sd_cmd::*};

// MUST be the first module listed
mod fmt;

/// Status for card in the ready state
pub const R1_READY_STATE: u8 = 0x00;
/// Status for card in the idle state
pub const R1_IDLE_STATE: u8 = 0x01;
/// Status bit for illegal command
pub const R1_ILLEGAL_COMMAND: u8 = 0x04;
/// Start data token for read or write single block*/
pub const DATA_START_BLOCK: u8 = 0xFE;
/// Stop token for write multiple blocks*/
pub const STOP_TRAN_TOKEN: u8 = 0xFD;
/// Start data token for write multiple blocks*/
pub const WRITE_MULTIPLE_TOKEN: u8 = 0xFC;
/// Mask for data response tokens after a write block operation
pub const DATA_RES_MASK: u8 = 0x1F;
/// Write data accepted token
pub const DATA_RES_ACCEPTED: u8 = 0x05;

#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// SD Card
pub struct Card {
    /// The type of this card
    pub card_type: CardCapacity,
    /// Operation Conditions Register
    pub ocr: OCR<SD>,
    /// Relative Card Address
    pub rca: u32,
    /// Card ID
    pub cid: CID<SD>,
    /// Card Specific Data
    pub csd: CSD<SD>,
}

impl Card {
    /// Size in bytes
    pub fn size(&self) -> u64 {
        // SDHC / SDXC / SDUC
        u64::from(self.csd.block_count()) * 512
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    ChipSelect,
    SpiError,
    Timeout,
    UnsupportedCard,
    Cmd58Error,
    Cmd59Error,
    RegisterError(u8),
    CrcMismatch(u16, u16),
    NotInitialized,
    WriteError,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::ChipSelect => write!(f, "ChipSelect error"),
            Error::SpiError => write!(f, "SPI error"),
            Error::Timeout => write!(f, "Timeout"),
            Error::UnsupportedCard => write!(f, "Unsupported card"),
            Error::Cmd58Error => write!(f, "CMD58 error"),
            Error::Cmd59Error => write!(f, "CMD59 error"),
            Error::RegisterError(r) => write!(f, "Register error: {}", r),
            Error::CrcMismatch(expected, actual) => {
                write!(f, "CRC mismatch: expected {}, actual {}", expected, actual)
            }
            Error::NotInitialized => write!(f, "Not initialized"),
            Error::WriteError => write!(f, "Write error"),
        }
    }
}

impl core::error::Error for Error {}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// High-level view of the card state in SPI mode
pub enum CardState {
    /// Card is in the ready/transfer state and can accept commands
    Ready,
    /// Card is in the idle state (often used as a low-power/sleep equivalent in SPI mode)
    Idle,
    /// Card reports another state or an error via the R1 status byte
    Error(u8),
}

/// Must be called between powerup and [SdSpi::init] to ensure the sdcard is properly initialized.
pub async fn sd_init<SPI, CS, BE>(spi: &mut SPI, cs: &mut CS) -> Result<(), Error>
where
    SPI: embedded_hal_async::spi::SpiBus<Error = BE>,
    CS: embedded_hal::digital::OutputPin,
{
    // Supply minimum of 74 clock cycles without CS asserted.
    cs.set_high().map_err(|_| Error::ChipSelect)?;
    spi.write(&[0xFF; 10]).await.map_err(|_| Error::SpiError)?;

    Ok(())
}

pub struct SdSpi<SPI, CS, D, ALIGN>
where
    SPI: embedded_hal_async::spi::SpiBus,
    CS: embedded_hal::digital::OutputPin,
    D: embedded_hal_async::delay::DelayNs,
    ALIGN: aligned::Alignment,
{
    spi: SPI,
    cs: CS,
    delay: D,
    card: Option<Card>,
    _align: PhantomData<ALIGN>,
}

impl<SPI, CS, D, ALIGN> SdSpi<SPI, CS, D, ALIGN>
where
    SPI: embedded_hal_async::spi::SpiBus,
    CS: embedded_hal::digital::OutputPin,
    D: embedded_hal_async::delay::DelayNs + Clone,
    ALIGN: aligned::Alignment,
{
    pub fn new(spi: SPI, cs: CS, delay: D) -> Self {
        Self {
            spi,
            cs,
            delay,
            card: None,
            _align: PhantomData,
        }
    }

    /// To comply with the SD card spec, [sd_init] must be called between powerup and calling this function.
    pub async fn init(&mut self) -> Result<(), Error> {
        with_timeout(self.delay.clone(), 1000, async {
            loop {
                let r = self.cmd(idle()).await?;
                if r == R1_IDLE_STATE {
                    return Ok(());
                }
            }
        })
        .await??;

        // "The SPI interface is initialized in the CRC OFF mode in default"
        // -- SD Part 1 Physical Layer Specification v9.00, Section 7.2.2 Bus Transfer Protection
        if self.cmd(cmd::<R1>(0x3B, 1)).await? != R1_IDLE_STATE {
            return Err(Error::Cmd59Error);
        }

        with_timeout(self.delay.clone(), 1000, async {
            loop {
                let (r, ext) = self.cmd_ext(send_if_cond(0x1, 0xAA)).await?;
                if r == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE) {
                    return Err(Error::UnsupportedCard);
                }
                if ext[3] == 0xAA {
                    return Ok(());
                }
            }
        })
        .await??;

        trace!("Valid card detected!");

        // If we get here we're at least a v2 card
        let mut card = Card::default();

        // send ACMD41
        with_timeout(self.delay.clone(), 1000, async {
            loop {
                let r = self.acmd(sd_send_op_cond(true, false, true, 0x20)).await?;
                if r == R1_READY_STATE {
                    return Ok(());
                }
            }
        })
        .await??;

        trace!("send_ocr");
        card.ocr = with_timeout(self.delay.clone(), 1000, async {
            loop {
                let (r, ext) = self.cmd_ext(cmd::<R3>(0x3A, 0)).await?;
                if r != R1_READY_STATE {
                    return Err(Error::Cmd58Error);
                }
                let ocr: OCR<SD> = u32::from_be_bytes(ext).into();
                if !ocr.is_busy() {
                    return Ok(ocr);
                }
            }
        })
        .await??;

        trace!("send_csd");
        let mut csd = [0xFFu8; 16];
        {
            self.cs.set_low().map_err(|_| Error::ChipSelect)?;
            let r1 = self.cmd_raw(send_csd(card.rca as u16)).await;
            let r2 = match r1 {
                Ok(r) if r == R1_READY_STATE => self.read_data_raw(&mut csd).await,
                Ok(r) => Err(Error::RegisterError(r)),
                Err(e) => Err(e),
            };
            self.cs.set_high().map_err(|_| Error::ChipSelect)?;
            r2?;
        }
        card.csd = u128::from_be_bytes(csd).into();

        trace!("all_send_cid");
        let mut cid = [0xFFu8; 16];
        {
            self.cs.set_low().map_err(|_| Error::ChipSelect)?;
            let r1 = self.cmd_raw(send_cid(card.rca as u16)).await;
            let r2 = match r1 {
                Ok(r) if r == R1_READY_STATE => self.read_data_raw(&mut cid).await,
                Ok(r) => Err(Error::RegisterError(r)),
                Err(e) => Err(e),
            };
            self.cs.set_high().map_err(|_| Error::ChipSelect)?;
            r2?;
        }
        card.cid = u128::from_be_bytes(cid).into();

        trace!("Card initialized: {:?}", card);
        debug!("Found card with size: {}bytes", card.size());

        self.card = Some(card);

        Ok(())
    }

    pub async fn read<const SIZE: usize>(
        &mut self,
        block_address: u32,
        data: &mut [Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Error> {
        self.cs.set_low().map_err(|_| Error::ChipSelect)?;
        let result = if data.len() == 1 {
            let r1 = self.cmd_raw(read_single_block(block_address)).await;
            match r1 {
                Err(e) => Err(e),
                Ok(r) if r != R1_READY_STATE => Err(Error::RegisterError(r)),
                Ok(_) => self.read_data_raw(&mut data[0][..]).await,
            }
        } else {
            let r1 = self.cmd_raw(read_multiple_blocks(block_address)).await;
            let r = match r1 {
                Err(e) => Err(e),
                Ok(r) if r != R1_READY_STATE => Err(Error::RegisterError(r)),
                Ok(_) => {
                    let mut result = Ok(());
                    for block in data.iter_mut() {
                        result = self.read_data_raw(&mut block[..]).await;
                        if result.is_err() {
                            break;
                        }
                    }
                    result
                }
            };
            // Stop transfer (CMD12) while CS is still LOW — spec-correct
            let _ = self.cmd_raw(stop_transmission()).await;
            r
        };
        self.cs.set_high().map_err(|_| Error::ChipSelect)?;
        result
    }

    pub async fn write<const SIZE: usize>(
        &mut self,
        block_address: u32,
        data: &[Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Error> {
        if data.len() == 1 {
            self.cs.set_low().map_err(|_| Error::ChipSelect)?;
            let result = {
                let r1 = self.cmd_raw(write_single_block(block_address)).await;
                match r1 {
                    Err(e) => Err(e),
                    Ok(r) if r != R1_READY_STATE => Err(Error::RegisterError(r)),
                    Ok(_) => {
                        let r = self.write_data_raw(DATA_START_BLOCK, &data[0][..]).await;
                        match r {
                            Err(e) => Err(e),
                            Ok(_) => {
                                let r = self.wait_idle_raw().await;
                                match r {
                                    Err(e) => Err(e),
                                    Ok(_) => {
                                        // check status, in SD SPI mode, the status is two bytes
                                        let cmd_status = match self.cmd_raw(sd_status()).await {
                                            Ok(r) => r,
                                            Err(_) => {
                                                trace!("Failed to read SD status after write");
                                                return Err(Error::WriteError);
                                            }
                                        };
                                        let _r2 = self.read_byte_raw().await?; // consume R2 second byte
                                        trace!("SD status after write: {}", cmd_status);
                                        if cmd_status != 0 {
                                            Err(Error::WriteError)
                                        } else {
                                            Ok(())
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            };
            self.cs.set_high().map_err(|_| Error::ChipSelect)?;
            result
        } else {
            self.cs.set_low().map_err(|_| Error::ChipSelect)?;
            let result = {
                // Try sending ACMD23 _before_ write.
                // This will pre-erase blocks to improve write performance.
                // Ignore errors — whether accepted or not, we proceed with the write.
                let _ = self.cmd_raw(app_cmd(self.card.map(|c| c.rca).unwrap_or(0) as u16)).await;
                let _ = self.cmd_raw(cmd::<R1>(0x17, data.len() as u32)).await;

                let r1 = self.cmd_raw(write_multiple_blocks(block_address)).await;
                match r1 {
                    Err(e) => Err(e),
                    Ok(r) if r != R1_READY_STATE => Err(Error::RegisterError(r)),
                    Ok(_) => {
                        let mut write_err: Option<Error> = None;
                        for block in data {
                            let r = self.wait_idle_raw().await;
                            if let Err(e) = r {
                                write_err = Some(e);
                                break;
                            }
                            let r = self.write_data_raw(WRITE_MULTIPLE_TOKEN, &block[..]).await;
                            if let Err(e) = r {
                                write_err = Some(e);
                                break;
                            }
                        }
                        if let Some(e) = write_err {
                            return Err(e);
                        }

                        // STOP_TRAN_TOKEN + mandatory stuff byte
                        let r = self.wait_idle_raw().await;
                        if let Err(e) = r { return Err(e); }
                        let r = self.spi.write(&[STOP_TRAN_TOKEN]).await.map_err(|_| Error::SpiError);
                        if let Err(e) = r { return Err(e); }
                        let mut stuff = [0xFFu8; 1];
                        let r = self.spi.transfer_in_place(&mut stuff).await.map_err(|_| Error::SpiError);
                        if let Err(e) = r { return Err(e); }

                        // Wait for all blocks to finish programming
                        self.wait_idle_raw().await
                    }
                }
            };
            self.cs.set_high().map_err(|_| Error::ChipSelect)?;
            result
        }
    }

    pub async fn size(&mut self) -> Result<u64, Error> {
        Ok(self.card.ok_or(Error::NotInitialized)?.size())
    }

    /// Returns a mutable reference to the underlying SpiBus.
    pub fn spi(&mut self) -> &mut SPI {
        &mut self.spi
    }

    // ── Raw primitives (CS managed by caller) ──────────────────────────────

    async fn read_byte_raw(&mut self) -> Result<u8, Error> {
        let mut buf = [0xFFu8; 1];
        self.spi
            .transfer_in_place(&mut buf)
            .await
            .map_err(|_| Error::SpiError)?;
        Ok(buf[0])
    }

    async fn wait_idle_raw(&mut self) -> Result<(), Error> {
        with_timeout(self.delay.clone(), 5000, async {
            while self.read_byte_raw().await? != 0xFF {}
            Ok(())
        })
        .await?
    }

    async fn read_data_raw(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        // Poll for start token — CS already LOW, no CS toggles
        let token = with_timeout(self.delay.clone(), 1000, async {
            loop {
                let b = self.read_byte_raw().await?;
                if b != 0xFF {
                    return Ok(b);
                }
            }
        })
        .await??;

        if token != DATA_START_BLOCK {
            return Err(Error::RegisterError(token));
        }

        // Bulk read via transfer_in_place (sends 0xFF, receives data)
        buffer.fill(0xFF);
        self.spi
            .transfer_in_place(buffer)
            .await
            .map_err(|_| Error::SpiError)?;

        // Read 2 CRC bytes
        let mut crc_bytes = [0xFFu8; 2];
        self.spi
            .transfer_in_place(&mut crc_bytes)
            .await
            .map_err(|_| Error::SpiError)?;
        let crc = u16::from_be_bytes(crc_bytes);
        let calc_crc = crc16(buffer);
        if crc != calc_crc {
            return Err(Error::CrcMismatch(crc, calc_crc));
        }

        Ok(())
    }

    async fn write_data_raw(&mut self, token: u8, buffer: &[u8]) -> Result<(), Error> {
        let crc = crc16(buffer).to_be_bytes();

        // Write token + data + CRC (CS already LOW from caller)
        self.spi
            .write(&[token])
            .await
            .map_err(|_| Error::SpiError)?;
        self.spi.write(buffer).await.map_err(|_| Error::SpiError)?;
        self.spi.write(&crc).await.map_err(|_| Error::SpiError)?;

        // Read data response token — still within the SAME CS-LOW window
        let status = with_timeout(self.delay.clone(), 1000, async {
            loop {
                let b = self.read_byte_raw().await?;
                if b != 0xFF {
                    return Ok(b);
                }
            }
        })
        .await??;

        if (status & DATA_RES_MASK) != DATA_RES_ACCEPTED {
            return Err(Error::WriteError);
        }

        Ok(())
    }

    async fn cmd_raw<R: Resp>(&mut self, command: Cmd<R>) -> Result<u8, Error> {
        if command.cmd != idle().cmd {
            self.wait_idle_raw().await?;
        }

        let mut buf = [
            0x40 | command.cmd,
            (command.arg >> 24) as u8,
            (command.arg >> 16) as u8,
            (command.arg >> 8) as u8,
            command.arg as u8,
            0,
        ];
        buf[5] = crc7(&buf[0..5]);

        self.spi.write(&buf).await.map_err(|_| Error::SpiError)?;

        // skip stuff byte for stop read
        if command.cmd == stop_transmission().cmd {
            let mut stuff = [0xFFu8; 1];
            self.spi
                .transfer_in_place(&mut stuff)
                .await
                .map_err(|_| Error::SpiError)?;
        }

        // Poll for R1 (first byte with MSB = 0)
        let byte = with_timeout(self.delay.clone(), 1000, async {
            loop {
                let byte = self.read_byte_raw().await?;
                if byte & 0x80 == 0 {
                    return Ok(byte);
                }
            }
        })
        .await??;

        Ok(byte)
    }

    /// Like `cmd_raw` but also reads 4 extension bytes atomically (for R3/R7 responses).
    async fn cmd_raw_ext<R: Resp>(&mut self, command: Cmd<R>) -> Result<(u8, [u8; 4]), Error> {
        let r1 = self.cmd_raw(command).await?;
        let mut ext = [0xFFu8; 4];
        self.spi
            .transfer_in_place(&mut ext)
            .await
            .map_err(|_| Error::SpiError)?;
        Ok((r1, ext))
    }

    // ── Managed commands (CS lifecycle handled internally) ──────────────────

    async fn wait_idle(&mut self) -> Result<(), Error> {
        self.cs.set_low().map_err(|_| Error::ChipSelect)?;
        let r = self.wait_idle_raw().await;
        self.cs.set_high().map_err(|_| Error::ChipSelect)?;
        r
    }

    async fn cmd<R: Resp>(&mut self, command: Cmd<R>) -> Result<u8, Error> {
        self.cs.set_low().map_err(|_| Error::ChipSelect)?;
        let r = self.cmd_raw(command).await;
        self.cs.set_high().map_err(|_| Error::ChipSelect)?;
        r
    }

    async fn cmd_ext<R: Resp>(&mut self, command: Cmd<R>) -> Result<(u8, [u8; 4]), Error> {
        self.cs.set_low().map_err(|_| Error::ChipSelect)?;
        let r = self.cmd_raw_ext(command).await;
        self.cs.set_high().map_err(|_| Error::ChipSelect)?;
        r
    }

    async fn acmd<R: Resp>(&mut self, command: Cmd<R>) -> Result<u8, Error> {
        self.cmd(app_cmd(self.card.map(|c| c.rca).unwrap_or(0) as u16))
            .await?;
        self.cmd(command).await
    }

    /// Put the card into idle state (CMD0 / GO_IDLE_STATE). Useful to re-initialize or low-power fallback.
    pub async fn enter_idle_state(&mut self) -> Result<(), Error> {
        let r1 = self.cmd(idle()).await?;
        if r1 & R1_IDLE_STATE != R1_IDLE_STATE {
            return Err(Error::RegisterError(r1));
        }
        Ok(())
    }

    /// Hint the card to enter sleep by stopping any ongoing transmission and releasing the bus.
    /// In SPI mode there is no dedicated SLEEP/AWAKE (CMD5 is SDIO). Practical low-power sequence is:
    /// - Ensure no data in flight
    /// - Send GO_IDLE_STATE if you want the card to enter idle
    /// - Then stop providing SPI clock and deassert CS at the platform level
    pub async fn sleep(&mut self) -> Result<(), Error> {
        // Wait for the card to be idle on the data line
        self.wait_idle().await?;
        // Put card into idle state to minimize power
        self.enter_idle_state().await?;
        Ok(())
    }

    /// Wake the card by providing 74+ clock cycles with CS deasserted, then re-run init() sequence.
    /// The user must have called sd_init(spi, cs) externally after power-up; here we only wake using clocks.
    /// For convenience, this just waits a bit so upper layers can provide clocks, then returns.
    pub async fn wake_hint(&mut self) -> Result<(), Error> {
        // Nothing to do at SPI transaction level; higher level must toggle CS high and provide clocks.
        // We insert a small delay to allow host to clock out dummy bytes.
        self.delay.delay_ms(1).await;
        Ok(())
    }

    /// Query the card for its current state in SPI mode using CMD13 (sd_status).
    /// Returns Ready when the R1 status is 0x00, Idle when R1 indicates the idle bit,
    /// otherwise Error(r1). In SPI mode, "sleeping" typically corresponds to Idle.
    pub async fn get_state(&mut self) -> Result<CardState, Error> {
        // CMD13 in SPI returns an R2 response: first byte is R1 status, second is extra status.
        // Both bytes must be read within the same CS window.
        self.cs.set_low().map_err(|_| Error::ChipSelect)?;
        let r1 = self.cmd_raw(sd_status()).await;
        let r2 = match r1 {
            Ok(r1) => {
                // Read and discard the second status byte to keep the bus aligned for next ops.
                let _r2_extra = self.read_byte_raw().await;
                Ok(r1)
            }
            Err(e) => Err(e),
        };
        self.cs.set_high().map_err(|_| Error::ChipSelect)?;
        let r1 = r2?;

        if r1 == R1_READY_STATE {
            Ok(CardState::Ready)
        } else if (r1 & R1_IDLE_STATE) != 0 {
            Ok(CardState::Idle)
        } else {
            Ok(CardState::Error(r1))
        }
    }

    /// Convenience helper to check if the card is currently in the idle (sleep-like) state.
    pub async fn is_idle(&mut self) -> Result<bool, Error> {
        Ok(matches!(self.get_state().await?, CardState::Idle))
    }
}

impl<SPI, CS, D, ALIGN, const SIZE: usize> block_device_driver::BlockDevice<SIZE>
    for SdSpi<SPI, CS, D, ALIGN>
where
    SPI: embedded_hal_async::spi::SpiBus,
    CS: embedded_hal::digital::OutputPin,
    D: embedded_hal_async::delay::DelayNs + Clone,
    ALIGN: aligned::Alignment,
{
    type Error = Error;
    type Align = ALIGN;

    async fn read(
        &mut self,
        block_address: u32,
        data: &mut [Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Self::Error> {
        self.read(block_address, data).await
    }

    async fn write(
        &mut self,
        block_address: u32,
        data: &[Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Self::Error> {
        self.write(block_address, data).await
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        self.size().await
    }
}

async fn with_timeout<D: embedded_hal_async::delay::DelayNs, F: Future>(
    mut delay: D,
    timeout: u32,
    fut: F,
) -> Result<F::Output, Error> {
    match select(fut, delay.delay_ms(timeout)).await {
        Either::First(r) => Ok(r),
        Either::Second(_) => Err(Error::Timeout),
    }
}

/// Perform the 7-bit CRC used on the SD card
fn crc7(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for mut d in data.iter().cloned() {
        for _bit in 0..8 {
            crc <<= 1;
            if ((d & 0x80) ^ (crc & 0x80)) != 0 {
                crc ^= 0x09;
            }
            d <<= 1;
        }
    }
    (crc << 1) | 1
}

/// Perform the X25 CRC calculation, as used for data blocks.
fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0u16;
    for &byte in data {
        crc = ((crc >> 8) & 0xFF) | (crc << 8);
        crc ^= u16::from(byte);
        crc ^= (crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xFF) << 5;
    }
    crc
}
