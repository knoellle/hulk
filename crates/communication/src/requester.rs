use anyhow::Context;
use futures_util::{stream::SplitSink, SinkExt};
use log::info;
use serde::Serialize;
use serde_json::Value;
use tokio::{net::TcpStream, sync::mpsc::Receiver};
use tokio_tungstenite::{tungstenite, MaybeTlsStream, WebSocketStream};

use super::CyclerOutput;

#[derive(Debug, Serialize)]
#[serde(tag = "type")]
pub enum Message {
    GetOutputHierarchy {
        id: usize,
    },
    SubscribeOutput {
        id: usize,
        output: CyclerOutput,
    },
    UnsubscribeOutput {
        id: usize,
        output: CyclerOutput,
    },
    GetParameterHierarchy {
        id: usize,
    },
    SubscribeParameter {
        id: usize,
        path: String,
    },
    UnsubscribeParameter {
        id: usize,
        path: String,
    },
    UpdateParameter {
        id: usize,
        path: String,
        data: Value,
    },
}

pub async fn requester(
    mut receiver: Receiver<Message>,
    mut writer: SplitSink<WebSocketStream<MaybeTlsStream<TcpStream>>, tungstenite::Message>,
) {
    while let Some(request) = receiver.recv().await {
        let request = serde_json::to_string(&request)
            .context("Serialization of Request type failed")
            .unwrap();
        writer
            .send(tungstenite::Message::Text(request))
            .await
            .context("Failed to send message to socket")
            .unwrap();
    }
    info!("Dropping requester, closing socket");
    writer.close().await.unwrap();
}
