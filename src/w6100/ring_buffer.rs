/// A byte-oriented circular buffer over a borrowed backing slice.
///
/// Tracks a read head and a length; the write position (`tail`) is derived.
/// Producers fill the buffer through the contiguous [`writable`](Self::writable)
/// region (handling wrap-around by writing in up to two passes); consumers
/// drain it with [`read`](Self::read).
pub(crate) struct RingBuffer<'a> {
    buf: &'a mut [u8],
    head: usize,
    len: usize,
}

impl<'a> RingBuffer<'a> {
    pub(crate) fn new(buf: &'a mut [u8]) -> Self {
        Self {
            buf,
            head: 0,
            len: 0,
        }
    }

    pub(crate) fn capacity(&self) -> usize {
        self.buf.len()
    }

    pub(crate) fn len(&self) -> usize {
        self.len
    }

    pub(crate) fn free(&self) -> usize {
        self.capacity() - self.len
    }

    fn tail(&self) -> usize {
        let cap = self.capacity();
        if cap == 0 { 0 } else { (self.head + self.len) % cap }
    }

    /// The largest contiguous run of free space available for writing right now.
    ///
    /// Because the buffer wraps, this may be shorter than [`free`](Self::free);
    /// call repeatedly (with [`advance_write`](Self::advance_write) in between)
    /// to fill the rest.
    pub(crate) fn writable(&mut self) -> &mut [u8] {
        if self.free() == 0 {
            return &mut self.buf[0..0];
        }

        let cap = self.capacity();
        let tail = self.tail();
        // When the data does not wrap (tail >= head), free space runs from
        // `tail` to the end of the slice; otherwise it runs `tail..head`.
        let end = if tail >= self.head { cap } else { self.head };

        &mut self.buf[tail..end]
    }

    /// Commit `n` bytes previously written into the [`writable`](Self::writable)
    /// region. `n` must not exceed that region's length.
    pub(crate) fn advance_write(&mut self, n: usize) {
        debug_assert!(n <= self.free());
        self.len += n;
    }

    /// Copy out up to `dst.len()` bytes, returning the number actually copied
    /// (which is `min(dst.len(), self.len())`).
    pub(crate) fn read(&mut self, dst: &mut [u8]) -> usize {
        let cap = self.capacity();
        let mut copied = 0;

        while copied < dst.len() && self.len > 0 {
            // Length of the contiguous readable run starting at `head`.
            let run = core::cmp::min(self.len, cap - self.head);
            let n = core::cmp::min(run, dst.len() - copied);

            dst[copied..copied + n].copy_from_slice(&self.buf[self.head..self.head + n]);

            self.head = (self.head + n) % cap;
            self.len -= n;
            copied += n;
        }

        copied
    }
}
