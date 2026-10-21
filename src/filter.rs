//! Standard and extended ID filter elements (FEATURES.md F2, F3), H7 only: written into the filter lists of the
//! instance's message RAM layout (SIDFC / XIDFC) in Config mode.
//!
//! Filters are checked in list order and the first enabled element that matches decides (Bosch M_CAN user
//! manual, "Acceptance Filtering"). Frames no element matches go where the [GlobalFilter](crate::config::GlobalFilter)
//! sends them. The index of the matching element is reported with each received frame
//! ([RxFrameHeader::filter_index](crate::RxFrameHeader)), counted in the standard or the extended list.

use crate::fdcan::{ConfigMode, Error, FdCan};
use crate::pac::message_ram::{
    ExtendedFilterElementF0, ExtendedFilterElementF1, StandardFilterConfiguration,
    StandardFilterElement, StandardFilterType,
};
use crate::{ExtendedId, StandardId};

/// What a filter element does with a frame it matches (SFEC / EFEC).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FilterAction {
    StoreInFifo0,
    StoreInFifo1,
    Reject,
    /// Flag a high-priority message (IR.HPM, HPMS) without storing it.
    SetPriority,
    SetPriorityAndStoreInFifo0,
    SetPriorityAndStoreInFifo1,
}

impl FilterAction {
    const fn config(self) -> StandardFilterConfiguration {
        match self {
            FilterAction::StoreInFifo0 => StandardFilterConfiguration::StoreInFIFO0,
            FilterAction::StoreInFifo1 => StandardFilterConfiguration::StoreInFIFO1,
            FilterAction::Reject => StandardFilterConfiguration::Reject,
            FilterAction::SetPriority => StandardFilterConfiguration::SetPriority,
            FilterAction::SetPriorityAndStoreInFifo0 => {
                StandardFilterConfiguration::SetPriorityAndStoreInFIFO0
            }
            FilterAction::SetPriorityAndStoreInFifo1 => {
                StandardFilterConfiguration::SetPriorityAndStoreInFIFO1
            }
        }
    }
}

/// Which IDs a filter element matches (SFT / EFT).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FilterMatch<I> {
    /// Every ID in `from..=to`; `from` must not be above `to`. Extended ranges ignore XIDAM (EFT = 11).
    Range { from: I, to: I },
    /// One of two IDs (the same twice for one ID).
    Dual(I, I),
    /// IDs with `received & mask == id & mask` (the classic filter).
    Mask { id: I, mask: I },
}

/// A standard (11-bit) ID filter element.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StandardFilter {
    pub matching: FilterMatch<StandardId>,
    pub action: FilterAction,
}

/// An extended (29-bit) ID filter element.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ExtendedFilter {
    pub matching: FilterMatch<ExtendedId>,
    pub action: FilterAction,
}

impl StandardFilter {
    /// The element word, `None` for a disabled element (all zero: SFEC = 000).
    pub(crate) fn element(filter: Option<&StandardFilter>) -> Result<u32, Error> {
        let Some(f) = filter else {
            return Ok(0);
        };
        let (sft, a, b) = match f.matching {
            FilterMatch::Range { from, to } => {
                if from.as_raw() > to.as_raw() {
                    return Err(Error::InvalidFilter);
                }
                (StandardFilterType::Range, from, to)
            }
            FilterMatch::Dual(a, b) => (StandardFilterType::DualID, a, b),
            FilterMatch::Mask { id, mask } => (StandardFilterType::Classic, id, mask),
        };
        Ok(StandardFilterElement::new()
            .with_sft(sft)
            .with_sfec(f.action.config())
            .with_sfid1(a.as_raw())
            .with_sfid2(b.as_raw())
            .into_bits())
    }
}

impl ExtendedFilter {
    /// The two element words, `None` for a disabled element (EFEC = 000).
    pub(crate) fn element(filter: Option<&ExtendedFilter>) -> Result<[u32; 2], Error> {
        let Some(f) = filter else {
            return Ok([0, 0]);
        };
        let (eft, a, b) = match f.matching {
            FilterMatch::Range { from, to } => {
                if from.as_raw() > to.as_raw() {
                    return Err(Error::InvalidFilter);
                }
                (0b11, from, to)
            }
            FilterMatch::Dual(a, b) => (0b01, a, b),
            FilterMatch::Mask { id, mask } => (0b10, id, mask),
        };
        let f0 = ExtendedFilterElementF0::new()
            .with_efec(f.action.config())
            .with_efid1(a.as_raw());
        let f1 = ExtendedFilterElementF1::new()
            .with_eft(eft)
            .with_efid2(b.as_raw());
        Ok([f0.into_bits(), f1.into_bits()])
    }
}

impl FdCan<ConfigMode> {
    /// Writes standard filter element `index` (`None` disables it). [Error::FilterIndexOutOfRange] if the layout
    /// has fewer standard filters. Apply the layout ([FdCan::set_layout] / [FdCan::apply_config]) first: a new
    /// layout zeroes its RAM, which disables every element.
    pub fn set_standard_filter(
        &mut self,
        index: u8,
        filter: Option<StandardFilter>,
    ) -> Result<(), Error> {
        let layout = &self.config.layout;
        if index >= layout.eleven_bit_filters_len {
            return Err(Error::FilterIndexOutOfRange);
        }
        let word = StandardFilter::element(filter.as_ref())?;
        let offset = layout.eleven_bit_filters_addr as usize + index as usize;
        // SAFETY: inside this instance's standard filter list, which lies inside the message RAM.
        unsafe {
            crate::pac::FDCAN_MSGRAM_ADDR
                .add(offset)
                .write_volatile(word)
        };
        Ok(())
    }

    /// Writes extended filter element `index` (`None` disables it), see [FdCan::set_standard_filter].
    pub fn set_extended_filter(
        &mut self,
        index: u8,
        filter: Option<ExtendedFilter>,
    ) -> Result<(), Error> {
        let layout = &self.config.layout;
        if index >= layout.twenty_nine_bit_filters_len {
            return Err(Error::FilterIndexOutOfRange);
        }
        let words = ExtendedFilter::element(filter.as_ref())?;
        let offset = layout.twenty_nine_bit_filters_addr as usize + 2 * index as usize;
        // SAFETY: inside this instance's extended filter list, which lies inside the message RAM.
        unsafe {
            let element = crate::pac::FDCAN_MSGRAM_ADDR.add(offset);
            element.write_volatile(words[0]);
            element.add(1).write_volatile(words[1]);
        }
        Ok(())
    }

    /// Number of standard and extended filter elements in the layout.
    pub fn filter_counts(&self) -> (u8, u8) {
        (
            self.config.layout.eleven_bit_filters_len,
            self.config.layout.twenty_nine_bit_filters_len,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn std_id(raw: u16) -> StandardId {
        StandardId::new(raw).unwrap()
    }
    fn ext_id(raw: u32) -> ExtendedId {
        ExtendedId::new(raw).unwrap()
    }

    /// Bosch M_CAN "Standard Message ID Filter Element": SFT [31:30], SFEC [29:27], SFID1 [26:16], SSYNC 15,
    /// SFID2 [10:0].
    #[test]
    fn standard_element() {
        assert_eq!(StandardFilter::element(None), Ok(0));
        let f = StandardFilter {
            matching: FilterMatch::Range {
                from: std_id(0x100),
                to: std_id(0x1FF),
            },
            action: FilterAction::StoreInFifo1,
        };
        assert_eq!(
            StandardFilter::element(Some(&f)),
            Ok(0b00 << 30 | 0b010 << 27 | 0x100 << 16 | 0x1FF)
        );
        let f = StandardFilter {
            matching: FilterMatch::Dual(std_id(0x7FF), std_id(0)),
            action: FilterAction::Reject,
        };
        assert_eq!(
            StandardFilter::element(Some(&f)),
            Ok(0b01 << 30 | 0b011 << 27 | 0x7FF << 16)
        );
        let f = StandardFilter {
            matching: FilterMatch::Mask {
                id: std_id(0x120),
                mask: std_id(0x7F0),
            },
            action: FilterAction::SetPriorityAndStoreInFifo0,
        };
        assert_eq!(
            StandardFilter::element(Some(&f)),
            Ok(0b10 << 30 | 0b101 << 27 | 0x120 << 16 | 0x7F0)
        );
        let f = StandardFilter {
            matching: FilterMatch::Range {
                from: std_id(2),
                to: std_id(1),
            },
            action: FilterAction::StoreInFifo0,
        };
        assert_eq!(StandardFilter::element(Some(&f)), Err(Error::InvalidFilter));
    }

    /// Bosch M_CAN "Extended Message ID Filter Element": F0 EFEC [31:29], EFID1 [28:0]; F1 EFT [31:30],
    /// EFID2 [28:0].
    #[test]
    fn extended_element() {
        assert_eq!(ExtendedFilter::element(None), Ok([0, 0]));
        let f = ExtendedFilter {
            matching: FilterMatch::Range {
                from: ext_id(0x1000),
                to: ext_id(0x1FFF_FFFF),
            },
            action: FilterAction::StoreInFifo0,
        };
        assert_eq!(
            ExtendedFilter::element(Some(&f)),
            Ok([0b001 << 29 | 0x1000, 0b11 << 30 | 0x1FFF_FFFF])
        );
        let f = ExtendedFilter {
            matching: FilterMatch::Dual(ext_id(1), ext_id(2)),
            action: FilterAction::SetPriority,
        };
        assert_eq!(
            ExtendedFilter::element(Some(&f)),
            Ok([0b100 << 29 | 1, 0b01 << 30 | 2])
        );
        let f = ExtendedFilter {
            matching: FilterMatch::Mask {
                id: ext_id(0x1234_5678),
                mask: ext_id(0x1FFF_FF00),
            },
            action: FilterAction::SetPriorityAndStoreInFifo1,
        };
        assert_eq!(
            ExtendedFilter::element(Some(&f)),
            Ok([0b110 << 29 | 0x1234_5678, 0b10 << 30 | 0x1FFF_FF00])
        );
        let f = ExtendedFilter {
            matching: FilterMatch::Range {
                from: ext_id(5),
                to: ext_id(4),
            },
            action: FilterAction::Reject,
        };
        assert_eq!(ExtendedFilter::element(Some(&f)), Err(Error::InvalidFilter));
    }
}
