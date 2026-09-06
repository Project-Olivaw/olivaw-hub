//! The one place lidar units become SLAM units.
//!
//! `ScanFrame` carries the RPLIDAR's Q6 degrees (clockwise) and Q2
//! millimetres; `olivaw-slam` wants metres in an x-forward, y-left frame.
//! Same convention as `olivaw_lidar::Point::to_cartesian`.

use olivaw_proto::ScanFrame;
use olivaw_slam::{Point2, ScanCloud};

/// Convert a rotation to a scan cloud, dropping no-return points.
pub fn scan_frame_to_cloud(frame: &ScanFrame, timestamp_ns: u64) -> ScanCloud {
    let points = frame
        .points
        .iter()
        .filter(|p| p.is_valid())
        .map(|p| {
            let r = f64::from(p.distance_mm()) / 1000.0;
            let a = f64::from(p.angle_deg()).to_radians();
            Point2::new(r * a.cos(), -r * a.sin())
        })
        .collect();
    ScanCloud::new(points, timestamp_ns)
}

#[cfg(test)]
mod tests {
    use super::*;
    use olivaw_proto::ScanPoint;

    #[test]
    fn forward_and_right_map_to_slam_frame() {
        let mut f = ScanFrame::default();
        f.points
            .push(ScanPoint {
                angle_q6: 0,
                dist_q2: 4000,
                quality: 40,
            })
            .unwrap(); // 1 m ahead
        f.points
            .push(ScanPoint {
                angle_q6: 90 * 64,
                dist_q2: 8000,
                quality: 40,
            })
            .unwrap(); // 2 m, 90° clockwise = right
        f.points
            .push(ScanPoint {
                angle_q6: 45 * 64,
                dist_q2: 0,
                quality: 0,
            })
            .unwrap(); // no return
        let cloud = scan_frame_to_cloud(&f, 7);
        assert_eq!(cloud.len(), 2);
        assert!((cloud.points[0].x - 1.0).abs() < 1e-6 && cloud.points[0].y.abs() < 1e-6);
        assert!(cloud.points[1].x.abs() < 1e-6 && (cloud.points[1].y + 2.0).abs() < 1e-6);
        assert_eq!(cloud.timestamp_ns, 7);
    }
}
