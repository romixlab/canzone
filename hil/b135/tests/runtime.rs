//! TX FIFO, remote frames, filter elements, going back to Config mode at run time, cancelling and the status
//! reader on FDCAN1 in internal loopback (FEATURES.md X2, X3, X9, F2, F3, M6, E7). Needs only the board and a
//! probe.
#![no_std]
#![no_main]

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use embassy_time::{Duration, Instant};
    use hil_b135::{Board, IDS, classic, pattern, receive};
    use mcan::config::{FrameTransmissionConfig, GlobalFilter, NonMatchingFilter};
    use mcan::fdcan::{Error, InternalLoopbackMode};
    use mcan::{
        DataFieldSize, ExtendedFilter, ExtendedId, FdCan, FdCanInstance, FilterAction, FilterMatch, Id,
        MessageRamLayout, NominalBitTiming, RxFifo, StandardFilter, StandardId, TxFrameHeader,
    };

    /// TX FIFO elements in the layout below.
    const TX_FIFO: u8 = 8;

    /// 4 standard and 2 extended filters, FIFO0 and FIFO1 with 8 elements of 64 bytes, no dedicated TX buffers,
    /// an 8-element TX FIFO.
    fn layout(board: &mut Board) -> MessageRamLayout {
        let builder = defmt::unwrap!(board.builder.take());
        let (layout, _) = defmt::unwrap!(
            builder
                .allocate_11bit_filters(4)
                .and_then(|b| b.allocate_29bit_filters(2))
                .and_then(|b| b.allocate_rx_fifo0_buffers(8, DataFieldSize::_64Bytes))
                .and_then(|b| b.allocate_rx_fifo1_buffers(8, DataFieldSize::_64Bytes))
                .map(|b| b.skip_dedicated_buffers())
                .and_then(|b| b.allocate_tx_event_fifo_buffers(0))
                .map(|b| b.tx_buffer_element_size(DataFieldSize::_64Bytes))
                .and_then(|b| b.allocate_fifo_or_queue(TX_FIFO))
                .and_then(|b| b.allocate_triggers(0))
                .ok()
        );
        layout
    }

    fn config(board: &mut Board) -> FdCan<mcan::ConfigMode> {
        let l = layout(board);
        let can = defmt::unwrap!(board.instances.take_enabled(FdCanInstance::FdCan1));
        let mut can = defmt::unwrap!(can.into_config_mode());
        can.set_nominal_bit_timing(defmt::unwrap!(NominalBitTiming::new(1, 47, 16, 16)));
        defmt::unwrap!(can.set_layout(l));
        can
    }

    /// Queues a frame in the TX FIFO, retrying while it is full.
    fn transmit(can: &mut FdCan<InternalLoopbackMode>, header: TxFrameHeader, data: &[u8]) {
        let deadline = Instant::now() + hil_b135::FRAME_TIMEOUT;
        loop {
            match can.transmit(header, data) {
                Err(Error::TxQueueFull) => {
                    defmt::assert!(Instant::now() < deadline, "TX FIFO stays full")
                }
                r => {
                    defmt::unwrap!(r);
                    return;
                }
            }
        }
    }

    fn wait_idle(can: &FdCan<InternalLoopbackMode>) {
        embassy_time::block_for(Duration::from_micros(1));
        let deadline = Instant::now() + hil_b135::FRAME_TIMEOUT * 8;
        while !can.is_idle() {
            defmt::assert!(Instant::now() < deadline, "TX did not complete");
        }
    }

    #[init]
    fn init() -> Board {
        Board::init()
    }

    /// Frames queued back to back in the TX FIFO go out in order, the FIFO reports full and its free level (X2).
    #[test]
    fn tx_fifo_keeps_order(mut board: Board) {
        let mut can = defmt::unwrap!(config(&mut board).into_internal_loopback());
        defmt::assert_eq!(can.tx_fifo_free_level(), TX_FIFO);
        // Fill the FIFO faster than the bus empties it: the 9th put finds it full.
        let mut queued = 0;
        while queued < 16 {
            match can.transmit(classic(IDS[queued % IDS.len()]), &pattern(queued, 8)[..8]) {
                Ok(_) => queued += 1,
                Err(Error::TxQueueFull) => break,
                Err(e) => defmt::panic!("transmit: {}", e),
            }
        }
        defmt::assert!(queued >= TX_FIFO as usize, "FIFO full after {} frames", queued);
        defmt::assert!(queued < 16);
        for n in 0..queued {
            let mut buf = [0u8; 8];
            let (h, copied) = receive(&mut can, RxFifo::Fifo0, &mut buf);
            defmt::assert_eq!((h.id, copied), (IDS[n % IDS.len()], 8));
            defmt::assert_eq!(buf, pattern(n, 8)[..8]);
        }
        wait_idle(&can);
        defmt::assert_eq!(can.tx_fifo_free_level(), TX_FIFO);
    }

    /// FD frames of every length through the TX FIFO; a frame longer than the element is refused (X2, X10).
    #[test]
    fn tx_fifo_fd_lengths(mut board: Board) {
        let mut c = config(&mut board);
        c.set_frame_transmit(FrameTransmissionConfig::AllowFdCanAndBRS);
        let mut can = defmt::unwrap!(c.into_internal_loopback());
        for (n, len) in [0, 8, 12, 16, 20, 24, 32, 48, 64].into_iter().enumerate() {
            let data = pattern(n, len);
            transmit(&mut can, TxFrameHeader::fd_brs(IDS[n % IDS.len()]), &data[..len]);
            let mut buf = [0u8; 64];
            let (h, copied) = receive(&mut can, RxFifo::Fifo0, &mut buf);
            defmt::assert_eq!((h.len as usize, copied), (len, len));
            defmt::assert!(h.bit_rate_switching);
            defmt::assert_eq!(buf[..len], data[..len]);
        }
        defmt::assert_eq!(
            can.transmit(TxFrameHeader::fd_brs(IDS[0]), &[0; 13]),
            Err(Error::WrongDataSize)
        );
    }

    /// Remote frames: RTR set, the DLC as requested, no data (X9).
    #[test]
    fn remote_frames(mut board: Board) {
        let mut can = defmt::unwrap!(config(&mut board).into_internal_loopback());
        for (n, id) in IDS.into_iter().enumerate() {
            let len = n % 9;
            transmit(&mut can, TxFrameHeader::remote(id), &[0; 8][..len]);
            let mut buf = [0u8; 8];
            let (h, _) = receive(&mut can, RxFifo::Fifo0, &mut buf);
            defmt::assert_eq!(h.id, id);
            defmt::assert!(h.rtr);
            defmt::assert_eq!(h.len as usize, len);
        }
        defmt::assert_eq!(
            can.transmit(TxFrameHeader::remote(IDS[0]), &[0; 12]),
            Err(Error::WrongDataSize)
        );
    }

    /// Standard and extended filter elements: FIFO routing, rejection, ranges, masks and the reported element
    /// index; frames no element matches go by the global filter (F2, F3).
    #[test]
    fn filter_elements(mut board: Board) {
        let mut c = config(&mut board);
        let sid = |raw| defmt::unwrap!(StandardId::new(raw));
        let eid = |raw| defmt::unwrap!(ExtendedId::new(raw));
        defmt::unwrap!(c.set_standard_filter(
            0,
            Some(StandardFilter {
                matching: FilterMatch::Dual(sid(0x100), sid(0x101)),
                action: FilterAction::Reject,
            })
        ));
        defmt::unwrap!(c.set_standard_filter(
            1,
            Some(StandardFilter {
                matching: FilterMatch::Range { from: sid(0x100), to: sid(0x1FF) },
                action: FilterAction::StoreInFifo1,
            })
        ));
        defmt::unwrap!(c.set_standard_filter(
            2,
            Some(StandardFilter {
                matching: FilterMatch::Mask { id: sid(0x520), mask: sid(0x7F0) },
                action: FilterAction::StoreInFifo0,
            })
        ));
        defmt::unwrap!(c.set_extended_filter(
            1,
            Some(ExtendedFilter {
                matching: FilterMatch::Mask { id: eid(0x1234_5600), mask: eid(0x1FFF_FF00) },
                action: FilterAction::StoreInFifo1,
            })
        ));
        defmt::assert_eq!(c.set_standard_filter(4, None), Err(Error::FilterIndexOutOfRange));
        defmt::assert_eq!(c.set_extended_filter(2, None), Err(Error::FilterIndexOutOfRange));
        c.set_global_filter(
            GlobalFilter::reject_all().set_handle_extended_frames(NonMatchingFilter::IntoRxFifo0),
        );
        let mut can = defmt::unwrap!(c.into_internal_loopback());

        // (id, FIFO, element index) for accepted frames, None for rejected ones
        let cases: [(Id, Option<(RxFifo, Option<u8>)>); 7] = [
            (Id::Standard(sid(0x100)), None),
            (Id::Standard(sid(0x102)), Some((RxFifo::Fifo1, Some(1)))),
            (Id::Standard(sid(0x1FF)), Some((RxFifo::Fifo1, Some(1)))),
            (Id::Standard(sid(0x52F)), Some((RxFifo::Fifo0, Some(2)))),
            (Id::Standard(sid(0x530)), None),
            (Id::Extended(eid(0x1234_56FF)), Some((RxFifo::Fifo1, Some(1)))),
            (Id::Extended(eid(0x1234_5700)), Some((RxFifo::Fifo0, None))),
        ];
        for (n, (id, expected)) in cases.into_iter().enumerate() {
            transmit(&mut can, classic(id), &pattern(n, 4)[..4]);
            wait_idle(&can);
            embassy_time::block_for(Duration::from_micros(200));
            match expected {
                None => {
                    defmt::assert_eq!(can.rx_fifo_fill_level(RxFifo::Fifo0), 0);
                    defmt::assert_eq!(can.rx_fifo_fill_level(RxFifo::Fifo1), 0);
                }
                Some((fifo, index)) => {
                    let mut buf = [0u8; 8];
                    let (h, _) = receive(&mut can, fifo, &mut buf);
                    defmt::assert_eq!(h.id, id);
                    defmt::assert_eq!(h.filter_index, index);
                }
            }
        }
    }

    /// Back to Config mode while running: new bit timing, filters kept, then on again (M6, T5).
    #[test]
    fn reconfigure_at_run_time(mut board: Board) {
        let mut c = config(&mut board);
        defmt::unwrap!(c.set_standard_filter(
            0,
            Some(StandardFilter {
                matching: FilterMatch::Dual(defmt::unwrap!(StandardId::new(0x7FF)), StandardId::ZERO),
                action: FilterAction::StoreInFifo1,
            })
        ));
        let mut can = defmt::unwrap!(c.into_internal_loopback());
        transmit(&mut can, classic(IDS[2]), &[1, 2]);
        let mut buf = [0u8; 8];
        defmt::assert_eq!(receive(&mut can, RxFifo::Fifo1, &mut buf).0.id, IDS[2]);

        let mut c = defmt::unwrap!(can.into_config_mode().map_err(|(e, _)| e));
        // 500 kbit/s
        c.set_nominal_bit_timing(defmt::unwrap!(NominalBitTiming::new(2, 47, 16, 16)));
        let mut can = defmt::unwrap!(c.into_internal_loopback());
        let start = Instant::now();
        transmit(&mut can, classic(IDS[2]), &[0; 8]);
        wait_idle(&can);
        // a classic frame with 8 bytes and an 11-bit ID is at least 111 bits: >= 222 µs at 500 kbit/s
        defmt::assert!(start.elapsed() >= Duration::from_micros(200));
        defmt::assert_eq!(receive(&mut can, RxFifo::Fifo1, &mut buf).0.id, IDS[2]);
        defmt::assert_eq!(can.tx_fifo_free_level(), TX_FIFO);

        // and into another mode: bus monitoring can't send
        let can = defmt::unwrap!(can.into_config_mode().map_err(|(e, _)| e));
        let can = defmt::unwrap!(can.into_bus_monitoring());
        defmt::assert!(!can.is_bus_off());
    }

    /// cancel_pending drops what the bus hasn't taken yet (X3).
    #[test]
    fn cancel_pending(mut board: Board) {
        let mut can = defmt::unwrap!(config(&mut board).into_internal_loopback());
        for n in 0..TX_FIFO as usize {
            defmt::unwrap!(can.transmit(classic(IDS[n % IDS.len()]), &[0; 8]));
        }
        embassy_time::block_for(Duration::from_micros(1));
        let cancelled = can.cancel_pending();
        defmt::assert!(cancelled >= TX_FIFO as u32 - 1, "{} cancelled", cancelled);
        wait_idle(&can);
        embassy_time::block_for(Duration::from_micros(500));
        // at most the frame that was on the bus arrives
        defmt::assert!(can.rx_fifo_fill_level(RxFifo::Fifo0) as u32 <= TX_FIFO as u32 - cancelled + 1);
        defmt::assert_eq!(can.tx_fifo_free_level(), TX_FIFO);
    }

    /// The status reader sees the same state as the owner, error codes once (E7).
    #[test]
    fn status_reader(mut board: Board) {
        let mut can = defmt::unwrap!(config(&mut board).into_internal_loopback());
        let reader = can.status_reader();
        transmit(&mut can, classic(IDS[1]), &[1]);
        wait_idle(&can);
        let s = reader.protocol_status();
        defmt::assert!(!s.bus_off && !s.error_passive);
        defmt::assert!(!s.last_error.is_error());
        defmt::assert_eq!(reader.error_counters().transmit, 0);
        defmt::assert!(!reader.is_bus_off());
        defmt::assert_eq!(can.protocol_status().bus_off, s.bus_off);
    }
}
