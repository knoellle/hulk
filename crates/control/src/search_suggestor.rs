use color_eyre::Result;
use context_attribute::context;
use coordinate_systems::{Field, Ground};
use framework::{AdditionalOutput, MainOutput};
use linear_algebra::{point, Isometry2, Point2};
use nalgebra::DMatrix;
use serde::{Deserialize, Serialize};
use types::{
    ball_position::{BallPosition, HypotheticalBallPosition},
    field_dimensions::FieldDimensions,
    parameters::SearchSuggestorParameters,
};

#[derive(Deserialize, Serialize)]
pub struct SearchSuggestor {
    heatmap_dimensions: (usize, usize),
    heatmap: DMatrix<f32>,
}

#[context]
pub struct CreationContext {
    field_dimensions: Parameter<FieldDimensions, "field_dimensions">,
    search_suggestor_configuration: Parameter<SearchSuggestorParameters, "search_suggestor">,
}

#[context]
pub struct CycleContext {
    search_suggestor_configuration: Parameter<SearchSuggestorParameters, "search_suggestor">,
    ball_position: Input<Option<BallPosition<Ground>>, "ball_position?">,
    invalid_ball_positions: Input<Vec<HypotheticalBallPosition<Ground>>, "invalid_ball_positions">,
    ground_to_field: Input<Option<Isometry2<Ground, Field>>, "ground_to_field?">,
    field_dimensions: Parameter<FieldDimensions, "field_dimensions">,
    heatmap: AdditionalOutput<DMatrix<f32>, "ball_search_heatmap">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub suggested_search_position: MainOutput<Option<Point2<Field>>>,
}

impl SearchSuggestor {
    pub fn new(_context: CreationContext) -> Result<Self> {
        let heatmap_dimensions = (
            _context.field_dimensions.length.round() as usize
                * _context.search_suggestor_configuration.cells_per_meter,
            _context.field_dimensions.width.round() as usize
                * _context.search_suggestor_configuration.cells_per_meter,
        );
        Ok(Self {
            heatmap_dimensions,
            heatmap: DMatrix::from_element(heatmap_dimensions.0, heatmap_dimensions.1, 0.0),
        })
    }

    pub fn cycle(&mut self, mut context: CycleContext) -> Result<MainOutputs> {
        self.update_heatmap(
            context.ball_position,
            context.invalid_ball_positions,
            context.ground_to_field.copied(),
            context.search_suggestor_configuration.cells_per_meter,
            context.search_suggestor_configuration.heatmap_decay_factor,
        );
        let maximum_heat_heatmap_position = self.heatmap.iamax_full();
        let mut suggested_search_position: Option<Point2<Field>> = None;
        if self.heatmap.get(maximum_heat_heatmap_position).is_some() {
            if self.heatmap[maximum_heat_heatmap_position]
                > context.search_suggestor_configuration.minimum_validity
            {
                let mut search_suggestion_x = maximum_heat_heatmap_position.0 as f32
                    / context.search_suggestor_configuration.cells_per_meter as f32;
                let mut search_suggestion_y = maximum_heat_heatmap_position.1 as f32
                    / context.search_suggestor_configuration.cells_per_meter as f32;
                let length_half = context.field_dimensions.length / 2.0;
                let width_half = context.field_dimensions.width / 2.0;

                if search_suggestion_x >= length_half {
                    search_suggestion_x -= length_half;
                } else {
                    search_suggestion_x = length_half - search_suggestion_x
                }
                if search_suggestion_y >= width_half {
                    search_suggestion_y -= width_half;
                } else {
                    search_suggestion_y = width_half - search_suggestion_y
                }

                search_suggestion_x +=
                    1.0 / context.search_suggestor_configuration.cells_per_meter as f32 / 2.0;
                search_suggestion_y +=
                    1.0 / context.search_suggestor_configuration.cells_per_meter as f32 / 2.0;

                suggested_search_position = Some(point![search_suggestion_x, search_suggestion_y]);
            }
        } else {
            println!("Invalid maximum heatmap position");
        }
        context.heatmap.fill_if_subscribed(|| self.heatmap.clone());

        Ok(MainOutputs {
            suggested_search_position: suggested_search_position.into(),
        })
    }

    fn update_heatmap(
        &mut self,
        ball_position: Option<&BallPosition<Ground>>,
        invalid_ball_positions: &Vec<HypotheticalBallPosition<Ground>>,
        ground_to_field: Option<Isometry2<Ground, Field>>,
        cells_per_meter: usize,
        heatmap_decay_factor: f32,
    ) {
        if let Some(ball_position) = ball_position {
            if let Some(ground_to_field) = ground_to_field {
                let ball_heatmap_position = self.calculate_heatmap_position(
                    cells_per_meter,
                    ground_to_field * ball_position.position,
                );
                if self.heatmap.get(ball_heatmap_position).is_some() {
                    self.heatmap[ball_heatmap_position] = 1.0;
                } else {
                    println!("Invalid ball heatmap position");
                }
            }
        }
        for ball_hypothesis in invalid_ball_positions {
            if let Some(ground_to_field) = ground_to_field {
                let heatmap_position = self.calculate_heatmap_position(
                    cells_per_meter,
                    ground_to_field * ball_hypothesis.position,
                );
                if self.heatmap.get(heatmap_position).is_some() {
                    self.heatmap[heatmap_position] =
                        (self.heatmap[heatmap_position] + ball_hypothesis.validity) / 2.0;
                } else {
                    println!("Invalid hypothesis heatmap position");
                }
            }
        }
        self.heatmap = self.heatmap.clone() * (1.0 - heatmap_decay_factor);
    }

    fn calculate_heatmap_position(
        &mut self,
        cells_per_meter: usize,
        hypothesis_position: Point2<Field>,
    ) -> (usize, usize) {
        let mut x_position: usize = 0;
        let mut y_position: usize = 0;
        if hypothesis_position.x() > 0.0 {
            x_position = (self.heatmap_dimensions.0 / 2)
                + (hypothesis_position.x() * cells_per_meter as f32).round() as usize;
        } else if hypothesis_position.x() < 0.0 {
            x_position = (self.heatmap_dimensions.0 / 2)
                - (hypothesis_position.x().abs() * cells_per_meter as f32).round() as usize;
        }
        if hypothesis_position.y() > 0.0 {
            y_position = (self.heatmap_dimensions.1 / 2)
                + (hypothesis_position.y() * cells_per_meter as f32).round() as usize;
        } else if hypothesis_position.y() < 0.0 {
            y_position = (self.heatmap_dimensions.1 / 2)
                - (hypothesis_position.y().abs() * cells_per_meter as f32).round() as usize;
        }
        if x_position >= 1 {
            x_position -= 1;
        }
        if y_position >= 1 {
            y_position -= 1;
        }
        x_position = x_position.clamp(0, self.heatmap_dimensions.0 - 1);
        y_position = y_position.clamp(0, self.heatmap_dimensions.1 - 1);

        (x_position, y_position)
    }
}
