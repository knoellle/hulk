// use std::any::TypeId;

// use bevy::{
//     asset::{HandleId, ReflectAsset},
//     prelude::*,
//     render::camera::Viewport,
// };
// use bevy_egui::{egui, EguiContext};
// use bevy_inspector_egui::{
//     bevy_inspector::{
//         by_type_id::{ui_for_asset, ui_for_resource},
//         hierarchy::{hierarchy_ui, SelectedEntities},
//         ui_for_entities_shared_components, ui_for_entity_with_children,
//     },
//     egui::Rect,
// };
// use bevy_reflect::TypeRegistry;
// use egui_dock::{DockArea, NodeIndex, Tree};

// use crate::MainCamera;

// pub struct InspectorUiPlugin;

// impl Plugin for InspectorUiPlugin {
//     fn build(&self, app: &mut App) {
//         app.insert_resource(InspectorSettings::default())
//             .insert_resource(UiState::new())
//             .add_system_to_stage(CoreStage::PreUpdate, inspector_ui.at_end())
//             .add_system(set_camera_viewport);
//     }
// }

// // pub fn ui_example_system(mut egui_context: ResMut<EguiContext>) {
// //     egui::Window::new("Hello").show(egui_context.ctx_mut(), |ui| {
// //         ui.label("world");
// //     });
// // }

// #[derive(Default, Resource)]
// pub struct InspectorSettings {
//     pub enabled: bool,
// }

// pub fn inspector_ui(world: &mut World) {
//     let inspector_settings = world.resource::<InspectorSettings>();
//     if !inspector_settings.enabled {
//         return;
//     }
//     let mut egui_context = world.resource_mut::<EguiContext>().ctx_mut().clone();

//     world.resource_scope::<UiState, _>(|world, mut ui_state| ui_state.ui(world, &mut egui_context));
// }

// #[derive(Eq, PartialEq)]
// enum InspectorSelection {
//     Entities,
//     Resource(TypeId, String),
//     Asset(TypeId, String, HandleId),
// }

// #[derive(Debug)]
// enum Window {
//     GameView,
//     Hierarchy,
//     Resources,
//     Assets,
//     Inspector,
// }

// #[derive(Resource)]
// struct UiState {
//     tree: Tree<Window>,
//     viewport_rect: Rect,
//     selected_entities: SelectedEntities,
//     selection: InspectorSelection,
// }

// impl UiState {
//     pub fn new() -> Self {
//         let mut tree = Tree::new(vec![Window::GameView]);
//         let [game, _inspector] = tree.split_right(NodeIndex::root(), 0.75, vec![Window::Inspector]);
//         let [game, _hierarchy] = tree.split_left(game, 0.2, vec![Window::Hierarchy]);
//         let [_game, _bottom] = tree.split_below(game, 0.8, vec![Window::Resources, Window::Assets]);

//         Self {
//             tree,
//             selected_entities: SelectedEntities::default(),
//             selection: InspectorSelection::Entities,
//             viewport_rect: egui::Rect::EVERYTHING,
//         }
//     }

//     fn ui(&mut self, world: &mut World, ctx: &mut egui::Context) {
//         let mut tab_viewer = TabViewer {
//             world,
//             viewport_rect: &mut self.viewport_rect,
//             selected_entities: &mut self.selected_entities,
//             selection: &mut self.selection,
//         };
//         DockArea::new(&mut self.tree).show(ctx, &mut tab_viewer);
//     }
// }

// struct TabViewer<'a> {
//     world: &'a mut World,
//     selected_entities: &'a mut SelectedEntities,
//     selection: &'a mut InspectorSelection,
//     viewport_rect: &'a mut egui::Rect,
// }

// impl egui_dock::TabViewer for TabViewer<'_> {
//     type Tab = Window;

//     fn ui(&mut self, ui: &mut egui::Ui, window: &mut Self::Tab) {
//         let type_registry = self.world.resource::<AppTypeRegistry>().0.clone();

//         match window {
//             Window::GameView => {
//                 (*self.viewport_rect, _) =
//                     ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
//             }
//             Window::Hierarchy => hierarchy_ui(self.world, ui, self.selected_entities),
//             Window::Resources => select_resource(ui, &type_registry.read(), self.selection),
//             Window::Assets => select_asset(ui, &type_registry.read(), self.world, self.selection),
//             Window::Inspector => match *self.selection {
//                 InspectorSelection::Entities => match self.selected_entities.as_slice() {
//                     &[entity] => ui_for_entity_with_children(self.world, entity, ui),
//                     entities => ui_for_entities_shared_components(self.world, entities, ui),
//                 },
//                 InspectorSelection::Resource(type_id, ref name) => {
//                     ui.label(name);
//                     ui_for_resource(self.world, type_id, ui, name, &type_registry.read())
//                 }
//                 InspectorSelection::Asset(type_id, ref name, handle) => {
//                     ui.label(name);
//                     ui_for_asset(self.world, type_id, handle, ui, &type_registry.read());
//                 }
//             },
//         }
//     }

//     fn title(&mut self, window: &mut Self::Tab) -> egui::WidgetText {
//         format!("{window:?}").into()
//     }

//     fn clear_background(&self, window: &Self::Tab) -> bool {
//         !matches!(window, Window::GameView)
//     }
// }

// fn select_resource(
//     ui: &mut egui::Ui,
//     type_registry: &TypeRegistry,
//     selection: &mut InspectorSelection,
// ) {
//     let mut resources: Vec<_> = type_registry
//         .iter()
//         .filter(|registration| registration.data::<ReflectResource>().is_some())
//         .map(|registration| (registration.short_name().to_owned(), registration.type_id()))
//         .collect();
//     resources.sort_by(|(name_a, _), (name_b, _)| name_a.cmp(name_b));

//     for (resource_name, type_id) in resources {
//         let selected = match *selection {
//             InspectorSelection::Resource(selected, _) => selected == type_id,
//             _ => false,
//         };

//         if ui.selectable_label(selected, &resource_name).clicked() {
//             *selection = InspectorSelection::Resource(type_id, resource_name);
//         }
//     }
// }

// fn select_asset(
//     ui: &mut egui::Ui,
//     type_registry: &TypeRegistry,
//     world: &World,
//     selection: &mut InspectorSelection,
// ) {
//     let mut assets: Vec<_> = type_registry
//         .iter()
//         .filter_map(|registration| {
//             let reflect_asset = registration.data::<ReflectAsset>()?;
//             Some((
//                 registration.short_name().to_owned(),
//                 registration.type_id(),
//                 reflect_asset,
//             ))
//         })
//         .collect();
//     assets.sort_by(|(name_a, ..), (name_b, ..)| name_a.cmp(name_b));

//     for (asset_name, asset_type_id, reflect_asset) in assets {
//         let mut handles: Vec<_> = reflect_asset.ids(world).collect();
//         handles.sort();

//         ui.collapsing(format!("{asset_name} ({})", handles.len()), |ui| {
//             for handle in handles {
//                 let selected = match *selection {
//                     InspectorSelection::Asset(_, _, selected_id) => selected_id == handle,
//                     _ => false,
//                 };

//                 if ui
//                     .selectable_label(selected, format!("{:?}", handle))
//                     .clicked()
//                 {
//                     *selection =
//                         InspectorSelection::Asset(asset_type_id, asset_name.clone(), handle);
//                 }
//             }
//         });
//     }
// }

// fn set_camera_viewport(
//     ui_state: Res<UiState>,
//     windows: Res<Windows>,
//     egui_settings: Res<bevy_egui::EguiSettings>,
//     mut cameras: Query<&mut Camera, With<MainCamera>>,
//     inspector_settings: Res<InspectorSettings>,
// ) {
//     let mut camera = cameras.single_mut();
//     if !inspector_settings.enabled {
//         camera.viewport = None;
//         return;
//     }

//     let window = windows.primary();
//     let scale_factor = window.scale_factor() * egui_settings.scale_factor;

//     let viewport_pos = ui_state.viewport_rect.left_top().to_vec2() * scale_factor as f32;
//     let viewport_size = ui_state.viewport_rect.size() * scale_factor as f32;

//     camera.viewport = Some(Viewport {
//         physical_position: UVec2::new(viewport_pos.x as u32, viewport_pos.y as u32),
//         physical_size: UVec2::new(viewport_size.x as u32, viewport_size.y as u32),
//         depth: 0.0..1.0,
//     });
// }
