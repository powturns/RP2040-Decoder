use dcc::Timer;
use embassy_time::Instant;

pub(crate) struct InstantTimer {
    mark: Option<Instant>,
}

impl InstantTimer {
    pub(crate) fn new() -> Self {
        Self { mark: None }
    }
}

impl Timer for InstantTimer {
    fn stop(&mut self) {
        self.mark = None;
    }

    fn start(&mut self) {
        self.mark = Some(Instant::now());
    }

    fn elapsed(&self) -> Option<usize> {
        self.mark.map(|m| m.elapsed().as_millis() as usize)
    }
}
