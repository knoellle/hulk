use std::sync::Arc;

use communication::client::CyclerOutput;
use eframe::egui::{Response, Ui, Widget};
use serde_json::Value;

use crate::{nao::Nao, panel::Panel};

pub struct SpectrumPanel {}

impl Widget for &mut SpectrumPanel {
    fn ui(self, _ui: &mut Ui) -> Response {
        todo!()
    }
}

impl Panel for SpectrumPanel {
    const NAME: &'static str = "Spectrum";

    fn new(_nao: Arc<Nao>, _value: Option<&Value>) -> Self {
        Self {}
    }
}
