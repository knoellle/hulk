use crate::{flow::context::Context, panels::TextPanel, SelectablePanel};
use color_eyre::Result;
use serde_json::{Number, Value};

pub fn run(context: Context) -> Result<Vec<SelectablePanel>> {
    let mut panels = Vec::new();

    for robot in context.robots {
        // let mut value =
        //     robot.subscribe_output("Control.additional_outputs.positions.head.pitch".parse()?);
        // value.map(|value| match value {
        //     Value::Number(angle) => {
        //         Value::Number(Number::from_f64(angle.as_f64().unwrap().to_degrees()).unwrap())
        //     }
        //     other => panic!("expected number, found: {other:?}"),
        // });
        //
        // panels.push(SelectablePanel::TextPanel(TextPanel::new_with_buffer(
        //     robot.clone(),
        //     value.clone(),
        // )));
        //
        // panels.push(SelectablePanel::TextPanel(TextPanel::new_with_buffer(
        //     robot, value,
        // )));
    }

    Ok(panels)
}
