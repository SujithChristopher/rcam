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

#[inline]
fn unpack_u8(src: &[u8], dst: &mut [u8]) {
    let mut s = 0;
    let mut d = 0;
    let n = dst.len();
    while d + 4 <= n && s + 5 <= src.len() {
        dst[d] = src[s];
        dst[d + 1] = src[s + 1];
        dst[d + 2] = src[s + 2];
        dst[d + 3] = src[s + 3];
        s += 5;
        d += 4;
    }
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

/// Number of AGC metering zones per side (the Pi's 15x15 weight grid).
const AGC_ZONES: usize = 15;
/// Stats are gathered from every `STATS_STEP`th row and column.
const STATS_STEP: usize = 2;

/// Y10P -> u8 through a 1024-entry lookup table, gathering the AGC histogram
/// on the way.
///
/// The LUT folds the whole Pi-style pixel pipeline (black level, digital
/// gain, gamma) into one table lookup, so it costs no more than the plain
/// high-byte unpack. The histogram is over the *raw* 10-bit codes - the
/// statistics must describe the sensor output before digital gain, as the Pi
/// frontend's do - with each sample counted `weights[zone]` times so the
/// centre-weighted metering falls out of the histogram directly. Zones follow
/// PiSP's layout: 15x15 even-sized cells centred on the frame, the remainder
/// border unweighted.
fn unpack_lut_stats(
    src: &[u8],
    dst: &mut [u8],
    w: usize,
    h: usize,
    lut: &[u8],
    weights: &[u8],
    hist: &mut [u32; 1024],
) {
    let stride = w * 10 / 8;
    let zw = (w / AGC_ZONES) & !1;
    let zh = (h / AGC_ZONES) & !1;
    let ox = ((w - AGC_ZONES * zw) / 2) & !1;
    let oy = ((h - AGC_ZONES * zh) / 2) & !1;
    // Column -> zone column, or AGC_ZONES when outside the grid.
    let colzone: Vec<usize> = (0..w)
        .map(|x| if x < ox || zw == 0 { AGC_ZONES } else { ((x - ox) / zw).min(AGC_ZONES) })
        .collect();
    let mut wrow = vec![0u32; w];
    let mut wrow_zone = usize::MAX;

    for y in 0..h {
        let s_row = y * stride;
        if s_row + stride > src.len() {
            break;
        }
        let srow = &src[s_row..s_row + stride];
        let drow = &mut dst[y * w..(y + 1) * w];
        let zy = if y < oy || zh == 0 { AGC_ZONES } else { ((y - oy) / zh).min(AGC_ZONES) };
        let stats = y % STATS_STEP == 0 && zy < AGC_ZONES;
        if stats && zy != wrow_zone {
            for x in 0..w {
                let zx = colzone[x];
                wrow[x] = if zx < AGC_ZONES && x % STATS_STEP == 0 {
                    weights[zy * AGC_ZONES + zx] as u32
                } else {
                    0
                };
            }
            wrow_zone = zy;
        }
        for (g, (s, d)) in srow.chunks_exact(5).zip(drow.chunks_exact_mut(4)).enumerate() {
            let lsb = s[4];
            for k in 0..4 {
                let v = ((s[k] as usize) << 2) | ((lsb >> (2 * k)) & 0x3) as usize;
                d[k] = lut[v];
                if stats {
                    hist[v] += wrow[4 * g + k];
                }
            }
        }
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
    fn grab_with<F>(&self, py: Python, f: F) -> PyResult<(Vec<u8>, i64, u32)>
    where
        F: FnOnce(&[u8], usize, usize) -> Vec<u8> + Send,
    {
        let fd = self.fd;
        let bufs = &self.buffers;
        let w = self.width;
        let h = self.height;
        py.allow_threads(move || -> io::Result<(Vec<u8>, i64, u32)> {
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

#[pymodule]
fn _native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Capture>()?;
    Ok(())
}
