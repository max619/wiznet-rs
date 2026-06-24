//! A lock-free single-producer / single-consumer byte ring.
//!
//! Replaces the old `&mut`-based `RingBuffer`. The two ends are reached through
//! `&self` (no exclusive borrow, no try-lock), so the producer and consumer can
//! live on opposite sides of the interrupt boundary without contending:
//!
//! - **rx ring** — produced by the chip-servicing interrupt (`run` /
//!   `dma_complete`), consumed by `main` via the socket handle's `read`.
//! - **tx ring** — produced by `main` via `write`, consumed by the interrupt.
//!
//! Each ring therefore has exactly one producer and one consumer; they
//! synchronize purely through the monotonic `head`/`tail` indices (an
//! `Acquire`/`Release` handshake) and never touch the same byte at once. The
//! backing buffer is reached through a raw pointer so the two ends can `memcpy`
//! disjoint regions without ever forming aliasing `&mut`s — the one place this
//! crate needs `unsafe`, mirroring `atomic_cell.rs`.
//!
//! The buffer is **installed once** (at socket open); until then the ring has
//! zero capacity and every `read`/`write` is a no-op. This matches the
//! "buffers handed out once, socket re-armed not reopened" rule.

use core::marker::PhantomData;
use core::sync::atomic::{AtomicPtr, AtomicUsize, Ordering};

/// A byte-oriented SPSC ring over a borrowed backing slice.
///
/// `head`/`tail` are free-running (wrapping) byte counters; the live length is
/// `tail - head` and the backing index is `counter % cap`. The producer only
/// ever advances `tail`, the consumer only `head`, so neither overwrites the
/// other's index.
pub(crate) struct SpscRing<'a> {
    ptr: AtomicPtr<u8>,
    cap: AtomicUsize,
    /// Consumer index — bytes taken out so far.
    head: AtomicUsize,
    /// Producer index — bytes put in so far.
    tail: AtomicUsize,
    _marker: PhantomData<&'a mut [u8]>,
}

impl<'a> SpscRing<'a> {
    /// An empty ring with no backing storage. [`install`](Self::install) gives it
    /// a buffer; until then `capacity`/`len`/`free` are 0 and I/O is a no-op.
    pub(crate) const fn new() -> Self {
        Self {
            ptr: AtomicPtr::new(core::ptr::null_mut()),
            cap: AtomicUsize::new(0),
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            _marker: PhantomData,
        }
    }

    /// Install the backing buffer (once, at socket open). The borrow's lifetime
    /// is tied to the ring, so the slice must outlive it.
    pub(crate) fn install(&self, buf: &'a mut [u8]) {
        let len = buf.len();
        self.head.store(0, Ordering::Relaxed);
        self.tail.store(0, Ordering::Relaxed);
        self.ptr.store(buf.as_mut_ptr(), Ordering::Relaxed);
        // Publish capacity last: a non-zero `cap` is the signal that the buffer
        // (stored above) is ready, so it carries `Release` to order those writes
        // ahead of any consumer/producer that observes the capacity.
        self.cap.store(len, Ordering::Release);
    }

    pub(crate) fn capacity(&self) -> usize {
        self.cap.load(Ordering::Acquire)
    }

    /// Bytes currently stored (available to read).
    pub(crate) fn len(&self) -> usize {
        let tail = self.tail.load(Ordering::Acquire);
        let head = self.head.load(Ordering::Acquire);
        tail.wrapping_sub(head)
    }

    /// Free space currently available to write.
    pub(crate) fn free(&self) -> usize {
        self.capacity() - self.len()
    }

    /// Producer: copy up to `src.len()` bytes in, returning the number stored
    /// (`min(src.len(), free())`). Wrap-around is handled in up to two `memcpy`s.
    pub(crate) fn write(&self, src: &[u8]) -> usize {
        let cap = self.capacity();
        if cap == 0 {
            return 0;
        }

        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Relaxed);
        let n = core::cmp::min(src.len(), cap - tail.wrapping_sub(head));
        if n == 0 {
            return 0;
        }

        let ptr = self.ptr.load(Ordering::Relaxed);
        let start = tail % cap;
        let first = core::cmp::min(n, cap - start);

        #[allow(unsafe_code)]
        // SAFETY: `ptr` backs `cap` initialized bytes; `start < cap` and the two
        // runs (`start..start+first`, then `0..n-first`) stay inside it and are
        // disjoint from the consumer's `head..` region. The `Release` on `tail`
        // below publishes these bytes before the consumer can observe them.
        unsafe {
            core::ptr::copy_nonoverlapping(src.as_ptr(), ptr.add(start), first);
            if n > first {
                core::ptr::copy_nonoverlapping(src.as_ptr().add(first), ptr, n - first);
            }
        }

        self.tail.store(tail.wrapping_add(n), Ordering::Release);
        n
    }

    /// Consumer: copy up to `dst.len()` bytes out, returning the number taken
    /// (`min(dst.len(), len())`). Wrap-around is handled in up to two `memcpy`s.
    pub(crate) fn read(&self, dst: &mut [u8]) -> usize {
        let cap = self.capacity();
        if cap == 0 {
            return 0;
        }

        let tail = self.tail.load(Ordering::Acquire);
        let head = self.head.load(Ordering::Relaxed);
        let n = core::cmp::min(dst.len(), tail.wrapping_sub(head));
        if n == 0 {
            return 0;
        }

        let ptr = self.ptr.load(Ordering::Relaxed);
        let start = head % cap;
        let first = core::cmp::min(n, cap - start);

        #[allow(unsafe_code)]
        // SAFETY: same backing as `write`; the read runs stay inside the buffer
        // and cover only bytes the producer published (it `Release`d `tail`,
        // acquired above), disjoint from the producer's `tail..` region.
        unsafe {
            core::ptr::copy_nonoverlapping(ptr.add(start), dst.as_mut_ptr(), first);
            if n > first {
                core::ptr::copy_nonoverlapping(ptr, dst.as_mut_ptr().add(first), n - first);
            }
        }

        self.head.store(head.wrapping_add(n), Ordering::Release);
        n
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{vec, vec::Vec};

    fn ring(cap: usize) -> SpscRing<'static> {
        let r = SpscRing::new();
        r.install(vec![0u8; cap].leak());
        r
    }

    #[test]
    fn empty_ring_is_noop() {
        let r = SpscRing::new();
        assert_eq!(r.capacity(), 0);
        assert_eq!(r.write(&[1, 2, 3]), 0);
        let mut buf = [0u8; 4];
        assert_eq!(r.read(&mut buf), 0);
    }

    #[test]
    fn write_then_read_roundtrip() {
        let r = ring(8);
        assert_eq!(r.free(), 8);
        assert_eq!(r.write(&[1, 2, 3, 4]), 4);
        assert_eq!(r.len(), 4);
        assert_eq!(r.free(), 4);

        let mut out = [0u8; 4];
        assert_eq!(r.read(&mut out), 4);
        assert_eq!(out, [1, 2, 3, 4]);
        assert_eq!(r.len(), 0);
    }

    #[test]
    fn write_saturates_at_capacity() {
        let r = ring(4);
        assert_eq!(
            r.write(&[1, 2, 3, 4, 5, 6]),
            4,
            "only `free` bytes accepted"
        );
        assert_eq!(r.len(), 4);
        assert_eq!(r.write(&[7]), 0, "full ring rejects");
    }

    #[test]
    fn read_is_bounded_by_stored_len() {
        let r = ring(8);
        r.write(&[1, 2, 3]);
        let mut out = [0u8; 8];
        assert_eq!(r.read(&mut out), 3);
        assert_eq!(&out[..3], &[1, 2, 3]);
    }

    /// The headline reason for the rewrite: producing and consuming around the
    /// buffer end must wrap correctly. Fill, drain part, then write across the
    /// seam and read it back contiguously.
    #[test]
    fn wraps_around_the_seam() {
        let r = ring(8);
        r.write(&[0, 1, 2, 3, 4, 5]); // tail = 6
        let mut sink = [0u8; 4];
        assert_eq!(r.read(&mut sink), 4); // head = 4
        assert_eq!(sink, [0, 1, 2, 3]);

        // free = 6; writing 5 bytes wraps (room 2 at end, 3 at start).
        assert_eq!(r.write(&[10, 11, 12, 13, 14]), 5);
        assert_eq!(r.len(), 7); // [4,5] + [10..15)

        let mut out = [0u8; 7];
        assert_eq!(r.read(&mut out), 7);
        assert_eq!(out, [4, 5, 10, 11, 12, 13, 14]);
    }

    /// Drive many small interleaved write/read steps across several wraps and
    /// confirm the byte stream is order-preserving and lossless (the SPSC
    /// contract, exercised single-threaded).
    #[test]
    fn streaming_preserves_order_across_wraps() {
        let r = ring(5);
        let mut next_write = 0u8;
        let mut next_read = 0u8;
        let mut drained: Vec<u8> = Vec::new();

        for _ in 0..100 {
            let chunk = [
                next_write,
                next_write.wrapping_add(1),
                next_write.wrapping_add(2),
            ];
            let w = r.write(&chunk);
            next_write = next_write.wrapping_add(w as u8);

            let mut out = [0u8; 2];
            let got = r.read(&mut out);
            drained.extend_from_slice(&out[..got]);
        }
        // flush the tail
        loop {
            let mut out = [0u8; 5];
            let got = r.read(&mut out);
            if got == 0 {
                break;
            }
            drained.extend_from_slice(&out[..got]);
        }

        for b in drained {
            assert_eq!(b, next_read, "bytes must come out in the order written");
            next_read = next_read.wrapping_add(1);
        }
    }
}
