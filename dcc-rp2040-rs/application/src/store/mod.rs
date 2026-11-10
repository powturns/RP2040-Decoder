//! CV store implementations.
//!
//! By default CVs are persisted with [`sequential_storage`] (wear-levelled, one map item per CV).
//! Enable the `cv-store-raw-flash` feature to instead persist the whole CV array to a single
//! flash sector, erasing and rewriting it on every write.

#[cfg(feature = "cv-store-raw-flash")]
mod flash;
#[cfg(not(feature = "cv-store-raw-flash"))]
mod ss;

pub use selected::{CvStore, new};

/// Offset of the `CV_FLASH` region from the start of flash (the stores take offsets, not addresses).
///
/// A function rather than a `const`, since `FLASH_BASE` is a pointer and can't be cast in const eval.
fn cv_flash_offset() -> u32 {
    crate::CV_FLASH_ORIGIN as u32 - embassy_rp::flash::FLASH_BASE as u32
}

#[cfg(feature = "cv-store-raw-flash")]
mod selected {
    use crate::AppFlash;
    use dcc::cv::store::Error;

    /// The CV store selected by the `cv-store-raw-flash` feature.
    pub type CvStore = super::flash::Flash<'static, embassy_rp::peripherals::FLASH, { crate::FLASH_SIZE }>;

    /// Creates the CV store in the `CV_FLASH` region, loading the persisted CVs.
    pub async fn new(flash: AppFlash) -> Result<CvStore, Error> {
        // The raw store sizes its region from the CV array, so it only needs the start.
        CvStore::new(flash, super::cv_flash_offset()).await
    }
}

#[cfg(not(feature = "cv-store-raw-flash"))]
mod selected {
    use crate::{AppFlash, CV_FLASH_SIZE};
    use dcc::cv::store::Error;

    /// Number of flash pages (erase sectors) available to the CV store.
    const CV_FLASH_PAGES: usize = CV_FLASH_SIZE / embassy_rp::flash::ERASE_SIZE;

    /// Number of keys whose flash locations are cached; speeds up lookups when writing CVs.
    const CV_KEY_POINTERS: usize = 16;

    /// The CV store selected by the `cv-store-raw-flash` feature.
    pub type CvStore =
        super::ss::SequentialStorage<AppFlash, CV_FLASH_PAGES, CV_FLASH_PAGES, CV_KEY_POINTERS>;

    /// Creates the CV store in the `CV_FLASH` region, loading the persisted CVs.
    pub async fn new(flash: AppFlash) -> Result<CvStore, Error> {
        let start = super::cv_flash_offset();
        CvStore::new(flash, start..start + CV_FLASH_SIZE as u32).await
    }
}
