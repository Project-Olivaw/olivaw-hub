//! Occupancy grid → 8-bit grayscale PNG (+ the PGM twin for ROS tools).

use olivaw_slam::OccupancyGrid;

/// A rendered map: pixels are `255 × (1 − p)`, so occupied is dark, unknown
/// is mid-gray, free is white. Row 0 is the top of the image (max y).
#[derive(Debug, Clone)]
pub struct MapSnapshot {
    /// Increments per snapshot.
    pub version: u64,
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// Bottom-left world corner, metres.
    pub origin: [f64; 2],
    /// Metres per pixel.
    pub resolution: f64,
    /// Row-major gray pixels, top row first.
    pub gray: Vec<u8>,
    /// PNG encoding of `gray`.
    pub png: Vec<u8>,
}

impl MapSnapshot {
    /// Render the grid.
    pub fn from_grid(grid: &OccupancyGrid, version: u64) -> anyhow::Result<Self> {
        let width = u32::try_from(grid.width())?;
        let height = u32::try_from(grid.height())?;
        let gray = to_gray(grid);
        let png = encode_png(width, height, &gray)?;
        let origin = grid.origin();
        Ok(Self {
            version,
            width,
            height,
            origin: [origin.x, origin.y],
            resolution: grid.resolution_m(),
            gray,
            png,
        })
    }

    /// Binary PGM (P5) with the `map_server` convention: occupied 0, free 254, unknown 205.
    pub fn to_pgm(&self) -> Vec<u8> {
        let mut out = format!("P5\n{} {}\n255\n", self.width, self.height).into_bytes();
        out.extend(self.gray.iter().map(|&g| match g {
            g if g < 90 => 0u8,
            g if g > 205 => 254,
            _ => 205,
        }));
        out
    }
}

/// Grid cells (row 0 = min y) → gray pixels (row 0 = max y).
fn to_gray(grid: &OccupancyGrid) -> Vec<u8> {
    let (w, h) = (grid.width(), grid.height());
    let cells = grid.cells();
    let mut gray = vec![205u8; w * h];
    for row in 0..h {
        let src = &cells[row * w..(row + 1) * w];
        let dst_row = h - 1 - row;
        for (col, &log_odds) in src.iter().enumerate() {
            let p = 1.0 / (1.0 + (-log_odds).exp());
            // Explicitly clamped before the cast, so no precision surprise.
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let g = ((1.0 - p) * 255.0).round().clamp(0.0, 255.0) as u8;
            gray[dst_row * w + col] = g;
        }
    }
    gray
}

fn encode_png(width: u32, height: u32, gray: &[u8]) -> anyhow::Result<Vec<u8>> {
    let mut out = Vec::with_capacity(gray.len() / 8);
    {
        let mut enc = png::Encoder::new(&mut out, width, height);
        enc.set_color(png::ColorType::Grayscale);
        enc.set_depth(png::BitDepth::Eight);
        enc.set_compression(png::Compression::Fast);
        let mut w = enc.write_header()?;
        w.write_image_data(gray)?;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use olivaw_slam::{GridConfig, Point2, Pose2, ScanCloud};

    #[test]
    fn renders_a_small_grid_with_a_wall() {
        let cfg = GridConfig {
            resolution_m: 0.1,
            width_cells: 40,
            height_cells: 40,
            origin: Point2::new(-2.0, -2.0),
            ..GridConfig::default()
        };
        let mut grid = OccupancyGrid::new(cfg).unwrap();
        let wall: Vec<Point2> = (0..20)
            .map(|i| Point2::new(1.5, -1.0 + f64::from(i) * 0.1))
            .collect();
        grid.integrate_scan(&Pose2::identity(), &ScanCloud::new(wall, 0));
        let snap = MapSnapshot::from_grid(&grid, 1).unwrap();
        assert_eq!((snap.width, snap.height), (40, 40));
        assert!(snap.png.starts_with(&[0x89, b'P', b'N', b'G']));
        // Something got darker than unknown, and something lighter (free beam).
        assert!(snap.gray.iter().any(|&g| g < 100));
        assert!(snap.gray.iter().any(|&g| g > 220));
        let pgm = snap.to_pgm();
        assert!(pgm.starts_with(b"P5\n40 40\n255\n"));
        assert_eq!(pgm.len(), b"P5\n40 40\n255\n".len() + 1600);
    }
}
