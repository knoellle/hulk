use bevy::{prelude::*, render::view::NoIndirectDrawing};
use bevy_mod_openxr::{add_xr_plugins, init::OxrInitPlugin, types::OxrExtensions};
use bevy_mujoco::MujocoVisualizerPlugin;

#[bevy_main]
fn main() {
    App::new()
        .add_plugins(add_xr_plugins(DefaultPlugins).set(OxrInitPlugin {
            exts: {
                let mut exts = OxrExtensions::default();
                exts.enable_fb_passthrough();
                exts.enable_hand_tracking();
                exts
            },
            ..default()
        }))
        .add_plugins(bevy_mod_xr::hand_debug_gizmos::HandGizmosPlugin)
        .add_plugins(MujocoVisualizerPlugin::new(None))
        .add_systems(Update, modify_camera)
        .insert_resource(AmbientLight {
            color: Default::default(),
            brightness: 500.0,
            affects_lightmapped_meshes: false,
        })
        .insert_resource(ClearColor(Color::NONE))
        .run();
}

#[derive(Component)]
struct CamModified;

fn modify_camera(
    cams: Query<Entity, (With<Camera>, Without<CamModified>)>,
    mut commands: Commands,
) {
    for cam in &cams {
        commands
            .entity(cam)
            .insert(Msaa::Off)
            .insert(NoIndirectDrawing)
            .insert(CamModified);
    }
}
