use std::{
    thread::spawn,
    time::{Duration, Instant, SystemTime},
};

use tokio::{runtime::Builder, select, sync::watch, time::sleep};

use crate::{execution::Replayer, ReplayerHardwareInterface};

#[derive(Clone, Copy)]
pub struct PlayerState {
    pub time: SystemTime,
    pub playing: bool,
    pub playback_rate: f32,
}

impl Default for PlayerState {
    fn default() -> Self {
        Self {
            time: SystemTime::UNIX_EPOCH,
            playing: false,
            playback_rate: 1.0,
        }
    }
}

pub fn spawn_workers(
    replayer: Replayer<ReplayerHardwareInterface>,
    sender: watch::Sender<PlayerState>,
    update_callback: impl Fn() + Send + Sync + 'static,
) {
    spawn(move || {
        let runtime = Builder::new_current_thread().enable_all().build().unwrap();

        runtime.spawn(playback_worker(sender.clone()));
        runtime.block_on(replay_worker(replayer, sender.subscribe(), update_callback));
    });
}

async fn replay_worker(
    mut replayer: Replayer<ReplayerHardwareInterface>,
    mut receiver: watch::Receiver<PlayerState>,
    update_callback: impl Fn() + Send + Sync + 'static,
) {
    let mut parameters_receiver = replayer.get_parameters_receiver();

    let mut merged_frame_times = replayer
        .get_recording_indices()
        .values()
        .enumerate()
        .flat_map(|(instance, index)| index.iter().map(move |timing| (instance, timing.timestamp)))
        .collect::<Vec<_>>();
    merged_frame_times.sort_by_key(|(_, time)| *time);
    let instance_names = replayer
        .get_recording_indices()
        .keys()
        .cloned()
        .collect::<Vec<_>>();
    let mut last_time = receiver.borrow().time;

    loop {
        select! {
            _ = parameters_receiver.wait_for_change() => {}
            _ = sleep(Duration::from_secs(1)) => {}
            result = receiver.changed() => {
                if result.is_err() {
                    // channel closed, quit
                    break;
                }
            }
        }

        let new_time = receiver.borrow().time;
        let end_index = match merged_frame_times.binary_search_by_key(&new_time, |item| item.1) {
            Ok(index) => index,
            Err(index) => index,
        }
        .min(merged_frame_times.len() - 1);

        let start_index =
            match merged_frame_times.binary_search_by_key(&last_time, |item| item.1) {
                Ok(index) => index,
                Err(index) => index,
            }
            // todo: replay other cyclers when going backwards
            .min(end_index);
        last_time = new_time;

        for (instance_index, time) in &merged_frame_times[start_index..=end_index] {
            let instance_name = &instance_names[*instance_index];
            if let Ok(Some(frame)) = replayer
                .get_recording_indices_mut()
                .get_mut(instance_name)
                .unwrap()
                .find_latest_frame_up_to(*time)
            {
                if let Err(error) = replayer.replay(instance_name, *time, &frame.data) {
                    eprintln!("{error:#?}");
                }
            }
        }

        update_callback()
    }
}

async fn playback_worker(sender: watch::Sender<PlayerState>) {
    let mut receiver = sender.subscribe();
    let mut last_autoplay_time = None;
    loop {
        select! {
            _ = receiver.changed() => {
                let state = *receiver.borrow();
                if !state.playing {
                    last_autoplay_time = None
                }
            }
            _ = sleep(Duration::from_millis(12)), if receiver.borrow().playing => {
                let elapsed = last_autoplay_time
                    .as_ref()
                    .map(Instant::elapsed)
                    .unwrap_or(Duration::from_millis(12));
                last_autoplay_time = Some(Instant::now());
                sender.send_modify(|state| {
                    if state.playback_rate.is_sign_positive(){
                        state.time += elapsed.mul_f32(state.playback_rate);
                    } else {
                        state.time -= elapsed.mul_f32(-state.playback_rate);
                    }
                });
            }
        }
    }
}
