//! rerun sink (`--rerun`, feature `rerun`): the same entities `olivaw-slam`'s
//! `slam_live` example logs, one namespace per car.

#![cfg_attr(
    not(feature = "rerun"),
    allow(clippy::unnecessary_wraps, clippy::unused_self)
)]

/// A cloneable handle to the recording, or a no-op without the feature.
#[derive(Clone)]
pub struct Sink {
    #[cfg(feature = "rerun")]
    rec: rerun::RecordingStream,
}

/// Spawn the viewer and connect. `Ok(None)` when built without the feature.
pub fn start() -> anyhow::Result<Option<Sink>> {
    #[cfg(feature = "rerun")]
    {
        let rec = rerun::RecordingStreamBuilder::new("olivaw_hub").spawn().map_err(|e| {
            anyhow::anyhow!("failed to spawn the rerun viewer ({e}); install it with `uv tool install rerun-sdk`")
        })?;
        Ok(Some(Sink { rec }))
    }
    #[cfg(not(feature = "rerun"))]
    {
        tracing::warn!("--rerun ignored: build with `--features rerun`");
        Ok(None)
    }
}

impl Sink {
    /// Log a tracked scan and the trajectory.
    #[allow(unused_variables)]
    pub fn log_tracked(
        &self,
        car: &str,
        seq: u32,
        scan_world: &[[f32; 2]],
        trajectory: &[(f32, f32)],
    ) {
        #[cfg(feature = "rerun")]
        {
            self.rec.set_time_sequence("scan", i64::from(seq));
            let _ = self.rec.log(
                format!("cars/{car}/scan"),
                &rerun::Points2D::new(scan_world.iter().map(|p| (p[0], p[1])))
                    .with_colors([rerun::Color::from_rgb(120, 200, 255)])
                    .with_radii([0.015]),
            );
            let _ = self.rec.log(
                format!("cars/{car}/trajectory"),
                &rerun::LineStrips2D::new([trajectory.to_vec()])
                    .with_colors([rerun::Color::from_rgb(255, 170, 40)]),
            );
        }
    }

    /// Log the occupancy grid.
    #[allow(unused_variables)]
    pub fn log_map(&self, car: &str, grid: &olivaw_slam::OccupancyGrid) {
        #[cfg(feature = "rerun")]
        {
            let _ = grid.log_to_rerun(&self.rec, &format!("cars/{car}/map"));
        }
    }
}
