//! Native V4L2 mmap capture for the OV9281 cameras.
//!
//! The hot path (blocking DQBUF + Y10P unpack + QBUF) runs inside
//! `Python::allow_threads`, so the GIL is released while waiting for and
//! processing a frame. The fd and mmap pointers are stored as plain integers
//! (Send), so two Python threads each driving their own `Capture` run truly in
//! parallel - unlike the subprocess+pipe path which is GIL/IPC bound.
//!
//! The Qualcomm CAMSS video node is a **multiplanar** capture device
//! (V4L2_CAP_VIDEO_CAPTURE_MPLANE), so this uses the `_MPLANE` buffer type,
//! `v4l2_pix_format_mplane`, and `v4l2_plane` arrays. The OV9281 stream is a
//! single plane (Y10P).
//!
//! Pipeline links/pad-formats and sensor controls are still configured from
//! Python (media-ctl / v4l2-ctl); this crate only owns the video-node capture.
#![allow(non_camel_case_types)]

use std::io;

use pyo3::exceptions::PyOSError;
use pyo3::prelude::*;
use pyo3::buffer::PyBuffer;
use pyo3::types::PyBytes;

// ---- V4L2 constants (Linux uapi, aarch64 LP64) --------------------------
const V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE: u32 = 9;
const V4L2_MEMORY_MMAP: u32 = 1;
const V4L2_FIELD_NONE: u32 = 1;
const NUM_PLANES: usize = 1; // OV9281 Y10P is a single plane

// _IOC(dir, type, nr, size): dir<<30 | size<<16 | type<<8 | nr
const fn ioc(dir: u32, ty: u32, nr: u32, size: u32) -> libc::c_ulong {
    (((dir << 30) | (size << 16) | (ty << 8) | nr) as u32) as libc::c_ulong
}
const VT: u32 = 0x56; // 'V'
const VIDIOC_S_FMT: libc::c_ulong = ioc(3, VT, 5, 208); // sizeof(v4l2_format)=208
const VIDIOC_REQBUFS: libc::c_ulong = ioc(3, VT, 8, 20);
const VIDIOC_QUERYBUF: libc::c_ulong = ioc(3, VT, 9, 88); // sizeof(v4l2_buffer)=88
const VIDIOC_QBUF: libc::c_ulong = ioc(3, VT, 15, 88);
const VIDIOC_DQBUF: libc::c_ulong = ioc(3, VT, 17, 88);
const VIDIOC_STREAMON: libc::c_ulong = ioc(1, VT, 18, 4);
const VIDIOC_STREAMOFF: libc::c_ulong = ioc(1, VT, 19, 4);

const fn fourcc(a: u8, b: u8, c: u8, d: u8) -> u32 {
    (a as u32) | ((b as u32) << 8) | ((c as u32) << 16) | ((d as u32) << 24)
}
const V4L2_PIX_FMT_Y10P: u32 = fourcc(b'Y', b'1', b'0', b'P');

#[repr(C)]
#[derive(Clone, Copy)]
struct v4l2_plane_pix_format {
    sizeimage: u32,
    bytesperline: u32,
    reserved: [u16; 6],
} // 20 bytes

#[repr(C)]
struct v4l2_pix_format_mplane {
    width: u32,
    height: u32,
    pixelformat: u32,
    field: u32,
    colorspace: u32,
    plane_fmt: [v4l2_plane_pix_format; 8], // VIDEO_MAX_PLANES = 8 -> 160 bytes
    num_planes: u8,
    flags: u8,
    enc: u8, // ycbcr_enc / hsv_enc union
    quantization: u8,
    xfer_func: u8,
    reserved: [u8; 7],
} // 20 + 160 + 5 + 7 = 192 bytes

#[repr(C)]
struct v4l2_format {
    type_: u32,
    _pad: u32, // union is 8-aligned -> fmt starts at offset 8
    pix_mp: v4l2_pix_format_mplane,
    _rest: [u8; 208 - 8 - 192], // pad the raw_data union to 200 bytes total
}

#[repr(C)]
#[derive(Clone, Copy)]
struct v4l2_requestbuffers {
    count: u32,
    type_: u32,
    memory: u32,
    capabilities: u32,
    flags: u8,
    reserved: [u8; 3],
}

#[repr(C)]
#[derive(Clone, Copy)]
struct v4l2_plane {
    bytesused: u32,
    length: u32,
    m_mem_offset: u64, // union m; for MMAP holds .mem_offset in the low 32 bits
    data_offset: u32,
    reserved: [u32; 11],
} // 4 + 4 + 8 + 4 + 44 = 64 bytes

#[repr(C)]
#[derive(Clone, Copy)]
struct v4l2_buffer {
    index: u32,
    type_: u32,
    bytesused: u32,
    flags: u32,
    field: u32,
    _pad0: u32,
    timestamp_sec: i64,
    timestamp_usec: i64,
    tc_type: u32,
    tc_flags: u32,
    tc_frames: u8,
    tc_seconds: u8,
    tc_minutes: u8,
    tc_hours: u8,
    tc_userbits: [u8; 4],
    sequence: u32,
    memory: u32,
    m_planes: u64, // union m; for MPLANE+MMAP holds a *v4l2_plane pointer
    length: u32,   // number of planes for the _MPLANE types
    reserved2: u32,
    request_fd: i32,
    _pad1: u32,
} // 88 bytes

unsafe fn xioctl<T>(fd: libc::c_int, req: libc::c_ulong, arg: *mut T) -> io::Result<()> {
    // Retry on EINTR.
    loop {
        let r = libc::ioctl(fd, req, arg);
        if r < 0 {
            let e = io::Error::last_os_error();
            if e.raw_os_error() == Some(libc::EINTR) {
                continue;
            }
            return Err(e);
        }
        return Ok(());
    }
}

fn pyerr(e: io::Error) -> PyErr {
    PyOSError::new_err(e.to_string())
}

/// Y10P -> u8 (the high byte of each pixel): keep 4 of every 5 bytes.
fn unpack_u8(src: &[u8], dst: &mut [u8]) {
    let groups = (dst.len() / 4).min(src.len() / 5);
    let done = unpack_u8_simd(&src[..groups * 5], &mut dst[..groups * 4]);
    for (s, d) in src[done * 5..groups * 5]
        .chunks_exact(5)
        .zip(dst[done * 4..groups * 4].chunks_exact_mut(4))
    {
        d.copy_from_slice(&s[..4]);
    }
}

/// NEON body of `unpack_u8`; returns how many 5-byte groups it converted.
///
/// 80 input bytes (16 groups, 64 px) per step: five 16-byte loads, then four
/// two-register table lookups each gather 16 high bytes from a 32-byte
/// window. The windows start 0/16/32/48 bytes in, so pixel groups beginning
/// at bytes 0/20/40/60 sit at offsets 0/4/8/12 within them - never reading
/// past the 80 bytes, so no slack is needed at the end of the buffer.
#[cfg(target_arch = "aarch64")]
fn unpack_u8_simd(src: &[u8], dst: &mut [u8]) -> usize {
    use std::arch::aarch64::*;
    const IDX: [u8; 16] = [0, 1, 2, 3, 5, 6, 7, 8, 10, 11, 12, 13, 15, 16, 17, 18];
    let steps = src.len() / 80;
    unsafe {
        let i0 = vld1q_u8(IDX.as_ptr());
        let i4 = vaddq_u8(i0, vdupq_n_u8(4));
        let i8 = vaddq_u8(i0, vdupq_n_u8(8));
        let i12 = vaddq_u8(i0, vdupq_n_u8(12));
        for n in 0..steps {
            let p = src.as_ptr().add(n * 80);
            let q = dst.as_mut_ptr().add(n * 64);
            let (a, b, c, d, e) = (
                vld1q_u8(p),
                vld1q_u8(p.add(16)),
                vld1q_u8(p.add(32)),
                vld1q_u8(p.add(48)),
                vld1q_u8(p.add(64)),
            );
            vst1q_u8(q, vqtbl2q_u8(uint8x16x2_t(a, b), i0));
            vst1q_u8(q.add(16), vqtbl2q_u8(uint8x16x2_t(b, c), i4));
            vst1q_u8(q.add(32), vqtbl2q_u8(uint8x16x2_t(c, d), i8));
            vst1q_u8(q.add(48), vqtbl2q_u8(uint8x16x2_t(d, e), i12));
        }
    }
    steps * 16
}

#[cfg(not(target_arch = "aarch64"))]
fn unpack_u8_simd(_src: &[u8], _dst: &mut [u8]) -> usize {
    0
}

#[inline]
fn unpack_u16_le(src: &[u8], dst: &mut [u8]) {
    // dst holds little-endian u16 (2 bytes/pixel)
    let mut s = 0;
    let mut d = 0; // pixel index
    let px = dst.len() / 2;
    while d + 4 <= px && s + 5 <= src.len() {
        let lsb = src[s + 4];
        for k in 0..4 {
            let v = ((src[s + k] as u16) << 2) | (((lsb >> (2 * k)) & 0x3) as u16);
            dst[2 * (d + k)] = v as u8;
            dst[2 * (d + k) + 1] = (v >> 8) as u8;
        }
        s += 5;
        d += 4;
    }
}

/// Address of a writable, C-contiguous Python buffer of exactly `len` items.
///
/// Returned as a usize so the write can happen with the GIL released; the
/// caller keeps the object alive for the duration of the call.
fn writable_ptr<T: pyo3::buffer::Element>(buf: &PyBuffer<T>, len: usize) -> PyResult<usize> {
    if buf.readonly() || !buf.is_c_contiguous() || buf.item_count() != len {
        return Err(pyo3::exceptions::PyValueError::new_err(format!(
            "output must be a writable C-contiguous buffer of {len} items"
        )));
    }
    Ok(buf.buf_ptr() as usize)
}

/// Number of AGC metering zones per side (the Pi's 15x15 weight grid).
const AGC_ZONES: usize = 15;
/// Stats come from every `STATS_ROWS`th row and every other pixel along it:
/// 1/8 of the frame, ~128k samples at 1280x800 - far more than the AGC's
/// 1024-bin histogram needs to be stable, at a fraction of the cost.
const STATS_ROWS: usize = 4;

/// Y10P -> u8 through a 1024-entry lookup table, gathering the AGC histogram
/// on the way.
///
/// The LUT folds the whole Pi-style pixel pipeline (black level, digital
/// gain, gamma) into one table lookup. The histogram is over the *raw* 10-bit
/// codes - the statistics must describe the sensor output before digital
/// gain, as the Pi frontend's do - with each sample counted `weights[zone]`
/// times so the centre-weighted metering falls out of the histogram directly.
/// Zones follow PiSP's layout: 15x15 even-sized cells centred on the frame,
/// the remainder border unweighted.
///
/// Speed: the tables are fixed-size arrays indexed by 10-bit values, so the
/// compiler drops every bounds check; the per-row weights are looked up per
/// 4-pixel group, not per pixel; and pixels 0 and 2 of each group count into
/// separate histograms so back-to-back increments of the same bin (flat image
/// areas) do not stall on each other.
fn unpack_lut_stats(
    src: &[u8],
    dst: &mut [u8],
    w: usize,
    h: usize,
    lut: &[u8],
    weights: &[u8],
    hist: &mut [u32; 1024],
) {
    let lut: &[u8; 1024] = lut.try_into().expect("lut is 1024 entries");
    let stride = w * 10 / 8;
    let groups = w / 4;
    let zw = (w / AGC_ZONES) & !1;
    let zh = (h / AGC_ZONES) & !1;
    let ox = ((w - AGC_ZONES * zw) / 2) & !1;
    let oy = ((h - AGC_ZONES * zh) / 2) & !1;
    let zone_of = |x: usize| {
        if zw == 0 || x < ox { AGC_ZONES } else { ((x - ox) / zw).min(AGC_ZONES) }
    };
    // Per 4-pixel group: the column zones of its pixels 0 and 2.
    let zcol: Vec<(usize, usize)> = (0..groups).map(|g| (zone_of(4 * g), zone_of(4 * g + 2))).collect();
    let mut wrow = vec![(0u32, 0u32); groups];
    let mut wrow_zone = usize::MAX;
    let mut h0 = [0u32; 1024];
    let mut h2 = [0u32; 1024];

    for y in 0..h {
        let s_row = y * stride;
        if s_row + stride > src.len() {
            break;
        }
        let srow = &src[s_row..s_row + groups * 5];
        let drow = &mut dst[y * w..y * w + groups * 4];
        let zy = if zh == 0 || y < oy { AGC_ZONES } else { ((y - oy) / zh).min(AGC_ZONES) };
        let pairs = srow.chunks_exact(5).zip(drow.chunks_exact_mut(4));

        if y % STATS_ROWS != 0 || zy >= AGC_ZONES {
            for (s, d) in pairs {
                let l = s[4] as usize;
                d[0] = lut[((s[0] as usize) << 2 | (l & 3)) & 1023];
                d[1] = lut[((s[1] as usize) << 2 | (l >> 2 & 3)) & 1023];
                d[2] = lut[((s[2] as usize) << 2 | (l >> 4 & 3)) & 1023];
                d[3] = lut[((s[3] as usize) << 2 | (l >> 6)) & 1023];
            }
            continue;
        }
        if zy != wrow_zone {
            let wz = &weights[zy * AGC_ZONES..(zy + 1) * AGC_ZONES];
            let wt = |zx: usize| if zx < AGC_ZONES { wz[zx] as u32 } else { 0 };
            for (wg, &(z0, z2)) in wrow.iter_mut().zip(&zcol) {
                *wg = (wt(z0), wt(z2));
            }
            wrow_zone = zy;
        }
        for ((s, d), &(w0, w2)) in pairs.zip(&wrow) {
            let l = s[4] as usize;
            let v0 = ((s[0] as usize) << 2 | (l & 3)) & 1023;
            let v2 = ((s[2] as usize) << 2 | (l >> 4 & 3)) & 1023;
            d[0] = lut[v0];
            d[1] = lut[((s[1] as usize) << 2 | (l >> 2 & 3)) & 1023];
            d[2] = lut[v2];
            d[3] = lut[((s[3] as usize) << 2 | (l >> 6)) & 1023];
            h0[v0] += w0;
            h2[v2] += w2;
        }
    }
    for ((o, a), b) in hist.iter_mut().zip(&h0).zip(&h2) {
        *o += a + b;
    }
}

#[pyclass]
struct Capture {
    fd: libc::c_int,
    buffers: Vec<(usize, usize)>, // (mmap ptr as usize, plane length)
    width: usize,
    height: usize,
}

#[pymethods]
impl Capture {
    #[new]
    #[pyo3(signature = (path, width, height, buffers = 4))]
    fn new(path: &str, width: usize, height: usize, buffers: u32) -> PyResult<Self> {
        let c_path = std::ffi::CString::new(path).map_err(|_| PyOSError::new_err("bad path"))?;
        let fd = unsafe { libc::open(c_path.as_ptr(), libc::O_RDWR) };
        if fd < 0 {
            return Err(pyerr(io::Error::last_os_error()));
        }
        let mut cap = Capture {
            fd,
            buffers: Vec::new(),
            width,
            height,
        };
        if let Err(e) = cap.init(buffers) {
            unsafe { libc::close(fd) };
            cap.fd = -1;
            return Err(pyerr(e));
        }
        Ok(cap)
    }

    fn next_raw<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let (out, _, _) = self.grab(py, 0)?;
        Ok(PyBytes::new(py, &out))
    }

    fn next_u8<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let (out, _, _) = self.grab(py, 1)?;
        Ok(PyBytes::new(py, &out))
    }

    fn next_u16<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let (out, _, _) = self.grab(py, 2)?;
        Ok(PyBytes::new(py, &out))
    }

    /// As next_raw/next_u8/next_u16, but also returning (timestamp_ns, sequence).
    fn next_raw_meta<'py>(&self, py: Python<'py>) -> PyResult<(Bound<'py, PyBytes>, i64, u32)> {
        let (out, ts, seq) = self.grab(py, 0)?;
        Ok((PyBytes::new(py, &out), ts, seq))
    }

    fn next_u8_meta<'py>(&self, py: Python<'py>) -> PyResult<(Bound<'py, PyBytes>, i64, u32)> {
        let (out, ts, seq) = self.grab(py, 1)?;
        Ok((PyBytes::new(py, &out), ts, seq))
    }

    /// Frame through the software ISP: returns
    /// (u8 pixels mapped through `lut`, timestamp_ns, sequence, histogram)
    /// where the histogram is 1024 little-endian u32 zone-weighted counts of
    /// the raw 10-bit codes. See `unpack_lut_stats`.
    fn next_isp<'py>(
        &self,
        py: Python<'py>,
        lut: &[u8],
        weights: &[u8],
    ) -> PyResult<(Bound<'py, PyBytes>, i64, u32, Bound<'py, PyBytes>)> {
        if lut.len() != 1024 || weights.len() != AGC_ZONES * AGC_ZONES {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "lut must be 1024 bytes and weights 225 bytes",
            ));
        }
        let lut = lut.to_vec();
        let weights = weights.to_vec();
        let mut hist = [0u32; 1024];
        let (out, ts, seq) = self.grab_with(py, |src, w, h| {
            let mut d = vec![0u8; w * h];
            unpack_lut_stats(src, &mut d, w, h, &lut, &weights, &mut hist);
            d
        })?;
        let hb: Vec<u8> = hist.iter().flat_map(|c| c.to_le_bytes()).collect();
        Ok((PyBytes::new(py, &out), ts, seq, PyBytes::new(py, &hb)))
    }

    /// next_u8, but unpacked straight into `out` (a reused H*W uint8 array)
    /// instead of a fresh allocation: returns (timestamp_ns, sequence).
    ///
    /// Allocating 1 MB per frame costs ~1 ms in page faults and zeroing -
    /// several times the unpack itself - so the hot path writes in place.
    fn next_u8_into(&self, py: Python<'_>, out: PyBuffer<u8>) -> PyResult<(i64, u32)> {
        let dst = writable_ptr(&out, self.width * self.height)?;
        let (_, ts, seq) = self.grab_with(py, move |src, w, h| {
            let d = unsafe { std::slice::from_raw_parts_mut(dst as *mut u8, w * h) };
            unpack_u8(src, d);
        })?;
        Ok((ts, seq))
    }

    /// next_isp, writing pixels into `out` (H*W uint8) and the histogram into
    /// `hist` (1024 uint32) in place: returns (timestamp_ns, sequence).
    fn next_isp_into(
        &self,
        py: Python<'_>,
        out: PyBuffer<u8>,
        hist: PyBuffer<u32>,
        lut: &[u8],
        weights: &[u8],
    ) -> PyResult<(i64, u32)> {
        if lut.len() != 1024 || weights.len() != AGC_ZONES * AGC_ZONES {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "lut must be 1024 bytes and weights 225 bytes",
            ));
        }
        let dst = writable_ptr(&out, self.width * self.height)?;
        let hp = writable_ptr(&hist, 1024)?;
        let (_, ts, seq) = self.grab_with(py, move |src, w, h| {
            let d = unsafe { std::slice::from_raw_parts_mut(dst as *mut u8, w * h) };
            let hh = unsafe { &mut *(hp as *mut [u32; 1024]) };
            hh.fill(0);
            unpack_lut_stats(src, d, w, h, lut, weights, hh);
        })?;
        Ok((ts, seq))
    }

    fn next_u16_meta<'py>(&self, py: Python<'py>) -> PyResult<(Bound<'py, PyBytes>, i64, u32)> {
        let (out, ts, seq) = self.grab(py, 2)?;
        Ok((PyBytes::new(py, &out), ts, seq))
    }

    /// Wait for the next frame and return only (timestamp_ns, sequence).
    ///
    /// The pixels are dropped without being copied out, which is what the
    /// frame-phase measurement wants: it needs the cadence, not the image, and
    /// at 1.28 MB/frame the copy would dominate.
    fn next_meta(&self, py: Python<'_>) -> PyResult<(i64, u32)> {
        let (_, ts, seq) = self.grab(py, 3)?;
        Ok((ts, seq))
    }

    fn close(&mut self) {
        self.teardown();
    }
}

impl Capture {
    fn init(&mut self, nbuf: u32) -> io::Result<()> {
        // S_FMT (multiplanar)
        let mut fmt: v4l2_format = unsafe { std::mem::zeroed() };
        fmt.type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.pix_mp.width = self.width as u32;
        fmt.pix_mp.height = self.height as u32;
        fmt.pix_mp.pixelformat = V4L2_PIX_FMT_Y10P;
        fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.pix_mp.num_planes = NUM_PLANES as u8;
        unsafe { xioctl(self.fd, VIDIOC_S_FMT, &mut fmt) }
            .map_err(|e| io::Error::new(e.kind(), format!("S_FMT: {e}")))?;

        // REQBUFS
        let mut req: v4l2_requestbuffers = unsafe { std::mem::zeroed() };
        req.count = nbuf;
        req.type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        req.memory = V4L2_MEMORY_MMAP;
        unsafe { xioctl(self.fd, VIDIOC_REQBUFS, &mut req) }
            .map_err(|e| io::Error::new(e.kind(), format!("REQBUFS: {e}")))?;
        if req.count < 1 {
            return Err(io::Error::new(io::ErrorKind::Other, "no buffers granted"));
        }

        // QUERYBUF + mmap + QBUF for each
        for i in 0..req.count {
            let mut planes: [v4l2_plane; NUM_PLANES] = unsafe { std::mem::zeroed() };
            let mut b: v4l2_buffer = unsafe { std::mem::zeroed() };
            b.type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            b.memory = V4L2_MEMORY_MMAP;
            b.index = i;
            b.length = NUM_PLANES as u32;
            b.m_planes = planes.as_mut_ptr() as u64;
            unsafe { xioctl(self.fd, VIDIOC_QUERYBUF, &mut b) }
                .map_err(|e| io::Error::new(e.kind(), format!("QUERYBUF: {e}")))?;

            let len = planes[0].length as usize;
            let offset = (planes[0].m_mem_offset & 0xffff_ffff) as libc::off_t;
            let ptr = unsafe {
                libc::mmap(
                    std::ptr::null_mut(),
                    len,
                    libc::PROT_READ | libc::PROT_WRITE,
                    libc::MAP_SHARED,
                    self.fd,
                    offset,
                )
            };
            if ptr == libc::MAP_FAILED {
                return Err(io::Error::last_os_error());
            }
            self.buffers.push((ptr as usize, len));

            unsafe { xioctl(self.fd, VIDIOC_QBUF, &mut b) }
                .map_err(|e| io::Error::new(e.kind(), format!("QBUF: {e}")))?;
        }

        // STREAMON
        let mut t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        unsafe { xioctl(self.fd, VIDIOC_STREAMON, &mut t) }
            .map_err(|e| io::Error::new(e.kind(), format!("STREAMON: {e}")))?;
        Ok(())
    }

    /// DQBUF one frame and return (pixels, buffer timestamp in ns, sequence).
    ///
    /// The timestamp is the one CAMSS stamps in its frame-done interrupt
    /// (`ts-monotonic, ts-src-eof` per VIDIOC_DQBUF), i.e. the same clock as
    /// Python's `time.monotonic_ns()` but taken in the kernel, so it carries
    /// ~100us of jitter instead of the milliseconds a userspace arrival time
    /// picks up. `sequence` is the driver's frame counter: a jump in it means
    /// the sensor really produced a frame that never reached us, as opposed to
    /// this thread merely having been descheduled.
    fn grab(&self, py: Python, mode: u8) -> PyResult<(Vec<u8>, i64, u32)> {
        self.grab_with(py, |src, w, h| match mode {
            0 => src.to_vec(),
            1 => {
                let mut d = vec![0u8; w * h];
                unpack_u8(src, &mut d);
                d
            }
            2 => {
                let mut d = vec![0u8; w * h * 2];
                unpack_u16_le(src, &mut d);
                d
            }
            _ => Vec::new(), // mode 3: timestamp/sequence only, no pixel copy
        })
    }

    /// DQBUF one frame, hand its packed bytes to `f` (GIL released), requeue.
    fn grab_with<F, R>(&self, py: Python, f: F) -> PyResult<(R, i64, u32)>
    where
        F: FnOnce(&[u8], usize, usize) -> R + Send,
        R: Send,
    {
        let fd = self.fd;
        let bufs = &self.buffers;
        let w = self.width;
        let h = self.height;
        py.allow_threads(move || -> io::Result<(R, i64, u32)> {
            let mut planes: [v4l2_plane; NUM_PLANES] = unsafe { std::mem::zeroed() };
            let mut b: v4l2_buffer = unsafe { std::mem::zeroed() };
            b.type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            b.memory = V4L2_MEMORY_MMAP;
            b.length = NUM_PLANES as u32;
            b.m_planes = planes.as_mut_ptr() as u64;
            unsafe { xioctl(fd, VIDIOC_DQBUF, &mut b)? };

            let idx = b.index as usize;
            let (ptr, len) = bufs[idx];
            let used = (planes[0].bytesused as usize).min(len);
            let src = unsafe { std::slice::from_raw_parts(ptr as *const u8, used) };

            let out = f(src, w, h);

            let ts_ns = b.timestamp_sec * 1_000_000_000 + b.timestamp_usec * 1_000;
            let seq = b.sequence;

            // requeue the same buffer
            unsafe { xioctl(fd, VIDIOC_QBUF, &mut b)? };
            Ok((out, ts_ns, seq))
        })
        .map_err(pyerr)
    }

    fn teardown(&mut self) {
        if self.fd < 0 {
            return;
        }
        let mut t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        unsafe {
            let _ = xioctl(self.fd, VIDIOC_STREAMOFF, &mut t);
            for &(ptr, len) in &self.buffers {
                libc::munmap(ptr as *mut libc::c_void, len);
            }
            libc::close(self.fd);
        }
        self.buffers.clear();
        self.fd = -1;
    }
}

impl Drop for Capture {
    fn drop(&mut self) {
        self.teardown();
    }
}

/// The Y10P -> u8 unpack on a caller-supplied packed frame (for tests).
#[pyfunction(name = "unpack_u8")]
fn py_unpack_u8<'py>(py: Python<'py>, src: &[u8], w: usize, h: usize) -> Bound<'py, PyBytes> {
    let mut d = vec![0u8; w * h];
    unpack_u8(src, &mut d);
    PyBytes::new(py, &d)
}

/// The ISP unpack on a caller-supplied packed frame (for tests):
/// returns (u8 pixels, 1024 little-endian u32 histogram).
#[pyfunction(name = "unpack_isp")]
fn py_unpack_isp<'py>(
    py: Python<'py>,
    src: &[u8],
    w: usize,
    h: usize,
    lut: &[u8],
    weights: &[u8],
) -> (Bound<'py, PyBytes>, Bound<'py, PyBytes>) {
    let mut d = vec![0u8; w * h];
    let mut hist = [0u32; 1024];
    unpack_lut_stats(src, &mut d, w, h, lut, weights, &mut hist);
    let hb: Vec<u8> = hist.iter().flat_map(|c| c.to_le_bytes()).collect();
    (PyBytes::new(py, &d), PyBytes::new(py, &hb))
}

#[pymodule]
fn _native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Capture>()?;
    m.add_function(wrap_pyfunction!(py_unpack_u8, m)?)?;
    m.add_function(wrap_pyfunction!(py_unpack_isp, m)?)?;
    Ok(())
}
