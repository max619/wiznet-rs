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

    /// Copy up to `src.len()` bytes into the buffer, returning the number
    /// actually stored (which is `min(src.len(), self.free())`).
    pub(crate) fn write(&mut self, src: &[u8]) -> usize {
        let cap = self.capacity();
        let mut written = 0;

        while written < src.len() && self.len < cap {
            let tail = (self.head + self.len) % cap;
            // Length of the contiguous free run starting at `tail`.
            let run = core::cmp::min(cap - self.len, cap - tail);
            let n = core::cmp::min(run, src.len() - written);

            self.buf[tail..tail + n].copy_from_slice(&src[written..written + n]);

            self.len += n;
            written += n;
        }

        written
    }

    /// The largest contiguous run of stored data available for reading right
    /// now. Because the buffer wraps, this may be shorter than [`len`](Self::len);
    /// call repeatedly (with [`advance_read`](Self::advance_read) in between) to
    /// drain the rest.
    pub(crate) fn readable(&self) -> &[u8] {
        if self.len == 0 {
            return &self.buf[0..0];
        }

        let cap = self.capacity();
        let run = core::cmp::min(self.len, cap - self.head);

        &self.buf[self.head..self.head + run]
    }

    /// Discard `n` bytes previously taken from the [`readable`](Self::readable)
    /// region. `n` must not exceed that region's length.
    pub(crate) fn advance_read(&mut self, n: usize) {
        debug_assert!(n <= self.len);
        let cap = self.capacity();
        self.head = (self.head + n) % cap;
        self.len -= n;
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
