use std::sync::Arc;

use anyhow::Result;
use communication::Cycler;
use eframe::{
    egui::{ComboBox, Ui},
    Storage,
};

use crate::{nao::Nao, twix_painter::TwixPainter};

use super::overlays::LineDetection;

pub trait Overlay {
    const NAME: &'static str;
    fn new(nao: Arc<Nao>, selected_cycler: Cycler) -> Self;
    fn paint(&self, painter: &TwixPainter) -> Result<()>;
}

pub struct EnabledOverlay<T>
where
    T: Overlay,
{
    nao: Arc<Nao>,
    overlay: Option<T>,
    active: bool,
}

impl<T> EnabledOverlay<T>
where
    T: Overlay,
{
    pub fn new(
        nao: Arc<Nao>,
        storage: Option<&dyn Storage>,
        active: bool,
        selected_cycler: Cycler,
    ) -> Self {
        let active = storage
            .and_then(|storage| storage.get_string(&format!("image.{}", T::NAME)))
            .and_then(|value| value.parse().ok())
            .unwrap_or(active);
        let layer = active.then(|| T::new(nao.clone(), selected_cycler));
        Self {
            nao,
            overlay: layer,
            active,
        }
    }

    pub fn update_cycler(&mut self, selected_cycler: Cycler) {
        if let Some(overlay) = self.overlay.as_mut() {
            *overlay = T::new(self.nao.clone(), selected_cycler);
        }
    }

    pub fn checkbox(&mut self, ui: &mut Ui, selected_cycler: Cycler) {
        if ui.checkbox(&mut self.active, T::NAME).changed() {
            match (self.active, self.overlay.is_some()) {
                (true, false) => self.overlay = Some(T::new(self.nao.clone(), selected_cycler)),
                (false, true) => self.overlay = None,
                _ => {}
            }
        }
    }

    pub fn paint(&self, painter: &TwixPainter) -> Result<()> {
        if let Some(layer) = &self.overlay {
            layer.paint(painter)?;
        }
        Ok(())
    }

    pub fn save(&self, storage: &mut dyn Storage) {
        storage.set_string(&format!("image.{}", T::NAME), self.active.to_string());
    }
}

pub struct Overlays {
    pub line_detection: EnabledOverlay<LineDetection>,
}

impl Overlays {
    pub fn new(nao: Arc<Nao>, storage: Option<&dyn Storage>, selected_cycler: Cycler) -> Self {
        let line_detection = EnabledOverlay::new(nao, storage, true, selected_cycler);
        Self { line_detection }
    }

    pub fn update_cycler(&mut self, selected_cycler: Cycler) {
        self.line_detection.update_cycler(selected_cycler);
    }

    pub fn combo_box(&mut self, ui: &mut Ui, selected_cycler: Cycler) {
        ComboBox::from_id_source("Overlays")
            .selected_text("Overlays")
            .show_ui(ui, |ui| {
                self.line_detection.checkbox(ui, selected_cycler);
            });
    }

    pub fn paint(&self, painter: &TwixPainter) -> Result<()> {
        let _ = self.line_detection.paint(painter);
        Ok(())
    }

    pub fn save(&self, storage: &mut dyn Storage) {
        self.line_detection.save(storage);
    }
}
