use std::sync::Arc;

use eframe::egui::Widget;
use serde_json::Value;

use crate::{nao::Nao, panel::Panel};

pub struct SpectrumPanel {}

impl Widget for &mut SpectrumPanel {
    fn ui(self, _ui: &mut eframe::egui::Ui) -> eframe::egui::Response {
        todo!()
    }
}

impl Panel for SpectrumPanel {
    const NAME: &'static str = "Spectrum";

    fn new(_nao: Arc<Nao>, _value: Option<&Value>) -> Self {
        Self {}
    }
}
