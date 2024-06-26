use std::{str::FromStr, sync::Arc};

use color_eyre::Result;
use eframe::epaint::{Color32, Stroke};

use communication::client::CyclerOutput;
use coordinate_systems::Ground;
use geometry::line::Line2;
use types::field_dimensions::FieldDimensions;

use crate::{
    nao::Nao, panels::map::layer::Layer, twix_painter::TwixPainter, value_buffer::ValueBuffer,
};

pub struct Lines {
    lines_in_ground_bottom: ValueBuffer,
    lines_in_ground_top: ValueBuffer,
}

impl Layer<Ground> for Lines {
    const NAME: &'static str = "Lines";

    fn new(nao: Arc<Nao>) -> Self {
        let lines_in_ground_bottom = nao
            .subscribe_output(CyclerOutput::from_str("VisionBottom.main.line_data.lines").unwrap());
        let lines_in_ground_top =
            nao.subscribe_output(CyclerOutput::from_str("VisionTop.main.line_data.lines").unwrap());
        Self {
            lines_in_ground_bottom,
            lines_in_ground_top,
        }
    }

    fn paint(
        &self,
        painter: &TwixPainter<Ground>,
        _field_dimensions: &FieldDimensions,
    ) -> Result<()> {
        let lines: Vec<Line2<Ground>> = [&self.lines_in_ground_bottom, &self.lines_in_ground_top]
            .iter()
            .filter_map(|buffer| buffer.parse_latest::<Vec<_>>().ok())
            .flatten()
            .collect();
        for line in lines {
            painter.line_segment(line.0, line.1, Stroke::new(0.04, Color32::RED));
        }
        Ok(())
    }
}
