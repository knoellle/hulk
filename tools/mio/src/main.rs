use std::f32::consts::PI;

use async_runtime::AsyncRuntimePlugin;
use ball::BallPlugin;
use bevy::{pbr::DirectionalLightShadowMap, prelude::*};
use bevy_obj::ObjPlugin;
use color_eyre::eyre::Result;
use field::FieldPlugin;
use nao::NaoPlugin;
use pan_orbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use parameters::Parameters;
use ui::UiPlugin;

mod async_runtime;
mod ball;
mod field;
mod inspector_ui;
mod nao;
mod pan_orbit_camera;
mod parameters;
mod ring;
mod ui;

fn main() -> Result<()> {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(ObjPlugin)
        .add_plugins(UiPlugin)
        .add_plugins(PanOrbitCameraPlugin)
        .insert_resource(Parameters::default())
        .insert_resource(DirectionalLightShadowMap { size: 4096 })
        .insert_resource(GizmoConfig {
            line_width: 4.0,
            ..default()
        })
        .add_plugins(BallPlugin)
        .add_plugins(FieldPlugin)
        .add_plugins(NaoPlugin)
        .add_plugins(AsyncRuntimePlugin)
        .add_systems(Startup, setup_camera)
        .add_systems(Startup, setup_light)
        .run();
    Ok(())
}

#[derive(Component)]
struct MainCamera;

fn setup_camera(mut commands: Commands) {
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_translation(Vec3::new(0.0, 6.0, 8.0))
                .looking_at(Vec3::ZERO, Vec3::Z),
            ..default()
        },
        PanOrbitCamera::default(),
        MainCamera,
    ));
}

fn setup_light(mut commands: Commands) {
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 0.05,
    });
    commands
        .spawn(Name::new("sun"))
        .insert(DirectionalLightBundle {
            directional_light: DirectionalLight {
                shadows_enabled: true,
                illuminance: 20000.0,
                ..default()
            },
            transform: Transform {
                translation: Vec3::new(0.0, 0.0, 2.0),
                rotation: Quat::from_euler(EulerRot::XYZ, -PI / 4.0, -PI / 6.0, 0.0),
                ..default()
            },
            ..default()
        });
}
