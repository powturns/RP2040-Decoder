use core::ops::Range;
use dcc::cv::CV_SIZE;
use dcc::cv::store::{Error, Store};
use embedded_storage_async::nor_flash::NorFlash;
use sequential_storage::{
    cache::{
        Cache, key_pointers::ArrayKeyPointers, page_pointers::ArrayPagePointers,
        page_states::ArrayPageStates,
    },
    map::{MapConfig, MapStorage},
};

// buf must be large enough to handle the largest stored value + the key size
// and must be at least as larget as embassy-rp's async flash ASYNC_READ_SIZE and WRITE_SIZE
// This gives a value of 4, since we only store single bytes and ASYNC_READ_SIZE is 4.
const DATA_BUFFER_SIZE: usize = 4;

/// Value reported for CVs that have never been written, matching erased flash.
///
/// [`dcc::cv::store::ensure_populated`] relies on this to detect an empty store.
const UNSET_CV: u8 = u8::MAX;

type CvMap<S, const PAGE_STATES: usize, const PAGE_POINTERS: usize, const KEY_POINTERS: usize> =
    MapStorage<
        u16,
        S,
        Cache<
            ArrayPageStates<PAGE_STATES>,
            ArrayPagePointers<PAGE_POINTERS>,
            ArrayKeyPointers<u16, KEY_POINTERS>,
            u16,
        >,
    >;

/// A [Store] that persists each CV as an individual `sequential_storage` map item, keyed by
/// its (1-indexed) CV address.
///
/// All CVs are mirrored in RAM: reads are served from the RAM copy, and writes go to flash
/// before updating it.
pub struct SequentialStorage<
    S: NorFlash,
    const PAGE_STATES: usize,
    const PAGE_POINTERS: usize,
    const KEY_POINTERS: usize,
> {
    storage: CvMap<S, PAGE_STATES, PAGE_POINTERS, KEY_POINTERS>,
    /// RAM copy of every CV; index 0 is CV 1.
    cache: [u8; CV_SIZE],
}

impl<S: NorFlash, const PAGE_STATES: usize, const PAGE_POINTERS: usize, const KEY_POINTERS: usize>
    SequentialStorage<S, PAGE_STATES, PAGE_POINTERS, KEY_POINTERS>
{
    /// Creates the store and loads all CVs persisted in `range` into RAM.
    ///
    /// NOTE: `range` is relative to the flash start, NOT an absolute address.
    pub async fn new(flash: S, range: Range<u32>) -> Result<Self, Error> {
        let mut storage = MapStorage::new(
            flash,
            MapConfig::new(range),
            Cache::new(
                ArrayPageStates::<PAGE_STATES>::new(),
                ArrayPagePointers::<PAGE_POINTERS>::new(),
                ArrayKeyPointers::<u16, KEY_POINTERS>::new(),
            ),
        );

        let cache = Self::load_cvs(&mut storage).await?;

        Ok(Self { storage, cache })
    }

    /// Reads every persisted CV from `storage`. CVs that were never written are [`UNSET_CV`].
    async fn load_cvs(
        storage: &mut CvMap<S, PAGE_STATES, PAGE_POINTERS, KEY_POINTERS>,
    ) -> Result<[u8; CV_SIZE], Error> {
        let mut cache = [UNSET_CV; CV_SIZE];
        let mut buf = [0u8; DATA_BUFFER_SIZE];

        let mut items = storage
            .fetch_all_items(&mut buf)
            .await
            .map_err(to_store_error)?;

        // Items are returned oldest first and a key can appear several times,
        // so later items overwrite earlier ones and the newest value wins.
        while let Some((address, value)) =
            items.next::<u8>(&mut buf).await.map_err(to_store_error)?
        {
            match cv_idx(address as usize) {
                Some(idx) => cache[idx] = value,
                None => warn!("ignoring stored item for invalid cv address {}", address),
            }
        }

        Ok(cache)
    }
}

impl<S: NorFlash, const PAGE_STATES: usize, const PAGE_POINTERS: usize, const KEY_POINTERS: usize>
    Store for SequentialStorage<S, PAGE_STATES, PAGE_POINTERS, KEY_POINTERS>
{
    async fn read_byte(&self, address: usize) -> Result<u8, Error> {
        cv_idx(address)
            .map(|idx| self.cache[idx])
            .ok_or(Error::InvalidAddress)
    }

    async fn read_bytes(&self, address: usize, len: usize) -> Result<&[u8], Error> {
        idx_range(address, len)
            .map(|range| &self.cache[range])
            .ok_or(Error::InvalidAddress)
    }

    async fn write_byte(&mut self, address: usize, value: u8) -> Result<(), Error> {
        self.write_bytes(address, &[value], false).await
    }

    async fn write_bytes(
        &mut self,
        address: usize,
        value: &[u8],
        force: bool,
    ) -> Result<(), Error> {
        // Calculate the cache index range and validate it so an invalid write doesn't persist a prefix.
        let idx_range = idx_range(address, value.len()).ok_or(Error::InvalidAddress)?;
        let mut buf = [0u8; DATA_BUFFER_SIZE];

        for (idx, &byte) in idx_range.zip(value) {
            if !force && self.cache[idx] == byte {
                continue;
            }

            // Key is the 1-indexed CV address; fits in u16 since CV_SIZE <= u16::MAX.
            let key = (idx + 1) as u16;
            self.storage
                .store_item(&mut buf, &key, &byte)
                .await
                .map_err(to_store_error)?;

            // Only update RAM once the value is persisted, so the cache never runs ahead of flash.
            self.cache[idx] = byte;
        }

        Ok(())
    }
}

/// Maps a 1-indexed CV address to its index in the cache.
fn cv_idx(address: usize) -> Option<usize> {
    address.checked_sub(1).filter(|&idx| idx < CV_SIZE)
}

/// Maps `len` CVs starting at the 1-indexed `address` to a range of cache indices.
fn idx_range(address: usize, len: usize) -> Option<Range<usize>> {
    let start = address.checked_sub(1)?;
    let end = start.checked_add(len)?;
    (end <= CV_SIZE).then_some(start..end)
}

/// Converts a `sequential_storage` error into a store [`Error`].
fn to_store_error<E>(error: sequential_storage::Error<E>) -> Error {
    match error {
        sequential_storage::Error::Storage { .. }
        | sequential_storage::Error::FullStorage
        | sequential_storage::Error::Corrupted { .. }
        | sequential_storage::Error::LogicBug { .. }
        | sequential_storage::Error::BufferTooBig
        | sequential_storage::Error::BufferTooSmall(_)
        | sequential_storage::Error::SerializationError(_)
        | sequential_storage::Error::ItemTooBig => Error::Io,
        // `sequential_storage::Error` is `#[non_exhaustive]`.
        _ => Error::Io,
    }
}
