//! Record every car event; replay it later with no hardware.
//!
//! `.olivawrec` = repeated `[u32 LE length][postcard(CarEvent)]` records.

pub mod recorder;
pub mod replay;
