use std::{str::FromStr, sync::Arc};

use communication::client::CyclerOutput;
use eframe::{
    egui::{
        plot::{Line, Plot, PlotPoints},
        Response, Ui, Widget,
    },
    epaint::Color32,
};
use log::error;
use serde_json::Value;
use types::whistle::DetectionInfo;

use crate::{nao::Nao, panel::Panel, value_buffer::ValueBuffer};

pub struct SpectrumPanel {
    spectrums: ValueBuffer,
    detection_infos: ValueBuffer,
}

const SPECTRUM_SUBSCRIPTION_KEY: &str = "Audio.additional_outputs.audio_spectrums";
const INFO_SUBSCRIPTION_KEY: &str = "Audio.additional_outputs.detection_infos";

impl Widget for &mut SpectrumPanel {
    fn ui(self, ui: &mut Ui) -> Response {
        let color = Color32::from_rgb(31, 119, 180);
        Plot::new(ui.id().with("value_plot"))
            .view_aspect(2.0)
            .show(ui, |plot_ui| {
                let line = self
                    .spectrums
                    .require_latest()
                    .map(|value: [Vec<[f64; 2]>; 4]| {
                        Line::new(PlotPoints::from(value[0].clone())).color(color)
                    });
                match line {
                    Ok(line) => plot_ui.line(line),
                    Err(error) => error!("{}", error),
                }
                let detection_info = self
                    .detection_infos
                    .require_latest()
                    .map(|detection_info: [DetectionInfo; 4]| detection_info[0].clone());
                match detection_info {
                    Ok(detection_info) => {
                        let index = detection_info.min_frequency_index;
                        let min_frequency_line_points = [[index as f64, 0.0], [index as f64, 0.5]];
                        let line = Line::new(PlotPoints::from_iter(min_frequency_line_points))
                            .color(Color32::RED);
                        plot_ui.line(line)
                    }
                    Err(error) => error!("{}", error),
                }
                // plot_ui.line(PlotPoints::from_iter(min_line))
            })
            .response
    }
}

impl Panel for SpectrumPanel {
    const NAME: &'static str = "Spectrum";

    fn new(nao: Arc<Nao>, _value: Option<&Value>) -> Self {
        let spectrum_output = CyclerOutput::from_str(SPECTRUM_SUBSCRIPTION_KEY).unwrap();
        let detection_output = CyclerOutput::from_str(INFO_SUBSCRIPTION_KEY).unwrap();
        let spectrums = nao.subscribe_output(spectrum_output);
        let detection_infos = nao.subscribe_output(detection_output);

        Self {
            spectrums,
            detection_infos,
        }
    }
}
