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

use crate::{nao::Nao, panel::Panel, value_buffer::ValueBuffer};

pub struct SpectrumPanel {
    values: ValueBuffer,
}

const SUBSCRIPTION_KEY: &'static str = "Audio.additional_outputs.audio_spectrums";

impl Widget for &mut SpectrumPanel {
    fn ui(self, ui: &mut Ui) -> Response {
        let color = Color32::from_rgb(31, 119, 180);
        Plot::new(ui.id().with("value_plot"))
            .view_aspect(2.0)
            .show(ui, |plot_ui| {
                let line = self
                    .values
                    .require_latest()
                    .map(|value: [Vec<[f64; 2]>; 4]| {
                        Line::new(PlotPoints::from(value[0].clone())).color(color)
                    });
                // .unwrap();
                // let test: Result<Vec<[f64; 2]>> = self.values.require_latest();
                // dbg!(test);
                match line {
                    Ok(line) => plot_ui.line(line),
                    Err(error) => error!("{}", error),
                }
            })
            .response
    }
}

impl Panel for SpectrumPanel {
    const NAME: &'static str = "Spectrum";

    fn new(nao: Arc<Nao>, _value: Option<&Value>) -> Self {
        let output = CyclerOutput::from_str(SUBSCRIPTION_KEY).unwrap();
        let values = nao.subscribe_output(output);

        Self { values }
    }
}
