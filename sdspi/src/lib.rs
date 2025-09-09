//! A crate for interfacing with SD cards over SPI.

#![no_std]

use aligned::Aligned;
use core::fmt::Debug;
use core::future::Future;
use core::marker::PhantomData;
use core::panic;
use embassy_futures::select::{select, Either};
use embedded_hal_async::spi::Operation;
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

pub struct SdSpi<SPI, D, ALIGN>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    D: embedded_hal_async::delay::DelayNs,
    ALIGN: aligned::Alignment,
{
    spi: SPI,
    delay: D,
    card: Option<Card>,
    _align: PhantomData<ALIGN>,
}

impl<SPI, D, ALIGN> SdSpi<SPI, D, ALIGN>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    D: embedded_hal_async::delay::DelayNs + Clone,
    ALIGN: aligned::Alignment,
{
    pub fn new(spi: SPI, delay: D) -> Self {
        Self {
            spi,
            delay,
            card: None,
            _align: PhantomData,
        }
    }

    /// To comply with the SD card spec, [sd_init] must be called between powerup and calling this function.
    pub async fn init(&mut self) -> Result<(), Error> {
        let r = async {
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
                    let r = self.cmd(send_if_cond(0x1, 0xAA)).await?;
                    if r == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE) {
                        return Err(Error::UnsupportedCard);
                    }
                    let mut buffer = [0xFFu8; 4];
                    self.spi
                        .transfer_in_place(&mut buffer[..])
                        .await
                        .map_err(|_| Error::SpiError)?;
                    if buffer[3] == 0xAA {
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
                    let r = self.cmd(cmd::<R3>(0x3A, 0)).await?;
                    if r != R1_READY_STATE {
                        return Err(Error::Cmd58Error);
                    }
                    let mut buffer = [0xFFu8; 4];
                    self.spi
                        .transfer_in_place(&mut buffer[..])
                        .await
                        .map_err(|_| Error::SpiError)?;
                    let ocr: OCR<SD> = u32::from_be_bytes(buffer).into();
                    if !ocr.is_busy() {
                        return Ok(ocr);
                    }
                }
            })
            .await??;

            trace!("send_csd");
            let r = self.cmd(send_csd(card.rca as u16)).await?;
            if r != R1_READY_STATE {
                return Err(Error::RegisterError(r));
            }
            let mut csd = [0xFFu8; 16];
            self.read_data(&mut csd).await?;
            card.csd = u128::from_be_bytes(csd).into();

            trace!("all_send_cid");
            let r = self.cmd(send_cid(card.rca as u16)).await?;
            if r != R1_READY_STATE {
                return Err(Error::RegisterError(r));
            }
            let mut cid = [0xFFu8; 16];
            self.read_data(&mut cid).await?;
            card.cid = u128::from_be_bytes(cid).into();

            trace!("Card initialized: {:?}", card);
            debug!("Found card with size: {}bytes", card.size());

            self.card = Some(card);

            Ok(())
        }
        .await;

        r
    }

    pub async fn read<const SIZE: usize>(
        &mut self,
        block_address: u32,
        data: &mut [Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Error> {
        let r = async {
            if data.len() == 1 {
                self.cmd(read_single_block(block_address)).await?;
                self.read_data(&mut data[0][..]).await?;
            } else {
                self.cmd(read_multiple_blocks(block_address)).await?;
                for block in data {
                    self.read_data(&mut block[..]).await?;
                }
                self.cmd(stop_transmission()).await?;
            }
            Ok(())
        }
        .await;

        r?;

        Ok(())
    }

    pub async fn write<const SIZE: usize>(
        &mut self,
        block_address: u32,
        data: &[Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Error> {
        let r = async {
            if data.len() == 1 {
                self.cmd(write_single_block(block_address)).await?;
                self.write_data(DATA_START_BLOCK, &data[0][..]).await?;
                self.wait_idle().await?;
                // check status, in SD SPI mode, the status is two bytes
                if self.cmd(sd_status()).await? != 0 {
                    return Err(Error::WriteError);
                }
                if self.read_byte().await? != 0 {
                    return Err(Error::WriteError);
                }
            } else {
                // Try sending ACMD23 _before_ write.
                // This will pre-erase blocks to improve write performance.
                // We ignore the return value, because whether its accepted
                // or not doesn't matter we will still proceed with the write
                self.acmd(cmd::<R1>(0x17, data.len() as u32)).await?;
                self.wait_idle().await?;

                self.cmd(write_multiple_blocks(block_address)).await?;
                for block in data {
                    self.wait_idle().await?;
                    self.write_data(WRITE_MULTIPLE_TOKEN, &block[..]).await?;
                }
                // stop the write
                self.wait_idle().await?;
                self.spi
                    .write(&[STOP_TRAN_TOKEN])
                    .await
                    .map_err(|_| Error::SpiError)?;
            }
            Ok(())
        }
        .await;

        r?;

        Ok(())
    }

    pub async fn size(&mut self) -> Result<u64, Error> {
        Ok(self.card.ok_or(Error::NotInitialized)?.size())
    }

    async fn read_data(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let r = with_timeout(self.delay.clone(), 1000, async {
            let mut byte = 0xFF;
            while byte == 0xFF {
                byte = self.read_byte().await?;
            }
            Ok(byte)
        })
        .await??;

        if r != DATA_START_BLOCK {
            return Err(Error::RegisterError(r));
        }

        buffer.fill(0xFF);
        self.spi
            .transfer_in_place(buffer)
            .await
            .map_err(|_| Error::SpiError)?;

        let mut crc_bytes = [0xFF; 2];
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

    async fn write_data(&mut self, token: u8, buffer: &[u8]) -> Result<(), Error> {
        self.spi
            .write(&[token])
            .await
            .map_err(|_| Error::SpiError)?;
        self.spi.write(buffer).await.map_err(|_| Error::SpiError)?;
        let crc_bytes = crc16(buffer).to_be_bytes();
        self.spi
            .write(&crc_bytes)
            .await
            .map_err(|_| Error::SpiError)?;

        let status = self.read_byte().await?;
        if (status & DATA_RES_MASK) != DATA_RES_ACCEPTED {
            return Err(Error::WriteError);
        }

        Ok(())
    }

    pub fn spi(&mut self) -> &mut SPI {
        &mut self.spi
    }

    async fn cmd<R: Resp>(&mut self, cmd: Cmd<R>) -> Result<u8, Error> {
        if cmd.cmd != idle().cmd {
            self.wait_idle().await?;
        }

        let mut buf = [
            0x40 | cmd.cmd,
            (cmd.arg >> 24) as u8,
            (cmd.arg >> 16) as u8,
            (cmd.arg >> 8) as u8,
            cmd.arg as u8,
            0,
        ];
        buf[5] = crc7(&buf[0..5]);

        self.spi.write(&buf).await.map_err(|_| Error::SpiError)?;

        // skip stuff byte for stop read
        if cmd.cmd == stop_transmission().cmd {
            self.spi
                .transfer_in_place(&mut [0xFF])
                .await
                .map_err(|_| Error::SpiError)?;
        }

        let byte = with_timeout(self.delay.clone(), 1000, async {
            loop {
                let byte = self.read_byte().await?;
                if byte & 0x80 == 0 {
                    return Ok(byte);
                }
            }
        })
        .await??;

        Ok(byte)
    }

    async fn acmd<R: Resp>(&mut self, cmd: Cmd<R>) -> Result<u8, Error> {
        self.cmd(app_cmd(self.card.map(|c| c.rca).unwrap_or(0) as u16))
            .await?;
        self.cmd(cmd).await
    }

    async fn wait_idle(&mut self) -> Result<(), Error> {
        with_timeout(self.delay.clone(), 5000, async {
            while self.read_byte().await? != 0xFF {}
            Ok(())
        })
        .await?
    }

    async fn read_byte(&mut self) -> Result<u8, Error> {
        let mut buf = [0xFFu8; 1];
        self.spi
            .transfer_in_place(&mut buf[..])
            .await
            .map_err(|_| Error::SpiError)?;

        Ok(buf[0])
    }

    /// Put the card into idle state (CMD0 / GO_IDLE_STATE). Useful to re-initialize or low-power fallback.
    pub async fn enter_idle_state(&mut self) -> Result<(), Error> {
        // CMD0 may be sent without waiting for idle clock cycles; we still respect wait_idle for consistency
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
        let r1 = self.cmd(sd_status()).await?;
        // Read and discard the second status byte to keep the bus aligned for next ops.
        let _r2_extra = self.read_byte().await?;

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

impl<SPI, D, ALIGN, const SIZE: usize> block_device_driver::BlockDevice<SIZE>
    for SdSpi<SPI, D, ALIGN>
where
    SPI: embedded_hal_async::spi::SpiDevice,
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
