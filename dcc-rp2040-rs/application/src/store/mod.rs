use core::mem::MaybeUninit;
use core::ops::{Deref, DerefMut, Range};
use dcc::cv::CV_SIZE;
use dcc::cv::store::{Error, Store};
use embassy_rp::flash::{Async, Flash as RpFlash, Instance};

/// A [Store] implementation that persists CVs to flash on the RP device.
pub struct Flash<'d, T: Instance, const FLASH_SIZE: usize> {
    /// The range of flash addresses used by cv data, relative to the start of the flash.
    cv_data_range: Range<u32>,
    flash: RpFlash<'d, T, Async, FLASH_SIZE>,
    cache: AlignedCvArray,
}

impl<'d, T: Instance, const FLASH_SIZE: usize> Flash<'d, T, FLASH_SIZE> {
    /// Create a new store that uses flash for persistence.
    ///
    /// NOTE: `offset` is an offset from the flash start, NOT an absolute address.
    pub async fn new(
        mut flash: RpFlash<'d, T, Async, FLASH_SIZE>,
        offset: u32,
    ) -> Result<Self, Error> {
        let cache = Self::read_cv_array(&mut flash, offset).await?;

        Ok(Self {
            flash,
            cv_data_range: offset..offset + (cache.len().max(embassy_rp::flash::ERASE_SIZE) as u32),
            cache,
        })
    }

    /// Writes the cached configuration variables to flash and reloads the latest values.
    fn sync_cvs(&mut self) -> Result<(), Error> {
        // since we keep the entire cache in memory and nothing else is stored in this flash sector, we can erase and write it to flash.
        // If we start storing additional items in flash (in the cv sector), we may need to read the entire sector (ERASE_SIZE) into memory, update the
        // cv values.

        let from = self.cv_data_range.start;
        let to = self.cv_data_range.end;

        debug!("erasing flash from {} to {}", from, to);

        self.flash.blocking_erase(from, to).map_err(|e| {
            error!("error erasing flash: {:?}", e);
            Error::Io
        })?;

        self.flash
            .blocking_write(from, self.cache.as_ref())
            .map_err(|e| {
                error!("error writing flash: {:?}", e);
                Error::Io
            })
    }

    /// Reads the configuration variables from flash into `data`.
    ///
    /// `data` must be aligned to a 4-byte boundary.
    async fn read_cv_array_aligned(
        flash: &mut RpFlash<'d, T, Async, FLASH_SIZE>,
        offset: u32,
        data: &mut [u8],
    ) -> Result<(), Error> {
        // verify alignment
        assert_eq!((data.as_ptr() as u32) % 4, 0);

        debug!(
            "reading {} bytes from flash at offset 0x{:x}...",
            data.len(),
            offset
        );

        flash.read(offset, data).await.map_err(|e| {
            error!("error reading flash: {:?}", e);
            Error::Io
        })?;

        Ok(())
    }

    /// Reads a new configuration variable array from flash.
    async fn read_cv_array(
        flash: &mut RpFlash<'d, T, Async, FLASH_SIZE>,
        offset: u32,
    ) -> Result<AlignedCvArray, Error> {
        // Allocate the cache without initializing it; we'll fill it directly from flash.
        let mut uninit_cache: MaybeUninit<AlignedCvArray> = MaybeUninit::uninit();

        let cache_ptr = uninit_cache.as_mut_ptr() as *mut u8;
        let raw_bytes: &mut [u8] = unsafe { core::slice::from_raw_parts_mut(cache_ptr, CV_SIZE) };

        Self::read_cv_array_aligned(flash, offset, raw_bytes).await?;

        // SAFETY: read_cvs wrote all CV_SIZE bytes into the cache.
        Ok(unsafe { uninit_cache.assume_init() })
    }
}

impl<'d, T: Instance, const FLASH_SIZE: usize> Store for Flash<'d, T, FLASH_SIZE> {
    fn read_byte(&self, address: usize) -> u8 {
        self.cache[address - 1]
    }

    fn read_bytes(&self, start: usize, len: usize) -> &[u8] {
        let start_idx = start - 1;
        let end_idx = start_idx + len;
        self.cache[start_idx..end_idx].as_ref()
    }

    fn write_byte(&mut self, address: usize, value: u8) -> Result<(), Error> {
        self.cache[address - 1] = value;
        self.write_bytes(address, &[value])
    }

    fn write_bytes(&mut self, start: usize, value: &[u8]) -> Result<(), Error> {
        let start_idx = start - 1;
        self.cache[start_idx..start_idx + value.len()].copy_from_slice(value);
        self.sync_cvs()
    }
}

#[repr(align(4))]
struct AlignedCvArray([u8; CV_SIZE]);

impl Deref for AlignedCvArray {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.0.as_ref()
    }
}

impl DerefMut for AlignedCvArray {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.as_mut()
    }
}
