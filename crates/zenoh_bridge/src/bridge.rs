use color_eyre::eyre::{bail, Result, WrapErr};
use futures_util::{pin_mut, StreamExt};
use ros2_client::{
    rustdds::{
        no_key::{DataReader, DefaultDecoder, DeserializerAdapter},
        RepresentationIdentifier,
    },
    Publisher,
};
use serde::{Deserialize, Serialize};
use std::fmt::Debug;
use zenoh::Session;

use crate::error::Error;

pub async fn forward_zenoh_to_ros<'a, T: Debug + Serialize + Deserialize<'a>>(
    zenoh_session: Session,
    zenoh_topic_name: &'static str,
    ros_publisher: Publisher<T>,
) -> Result<()> {
    let zenoh_subscriber = zenoh_session
        .declare_subscriber(format!("booster/{zenoh_topic_name}"))
        .await
        .map_err(Error::Zenoh)
        .wrap_err("failed to create Zenoh subscriber")?;

    while let Ok(message) = zenoh_subscriber.recv_async().await {
        let deserialized_message =
            cdr::deserialize(&message.payload().to_bytes()).wrap_err("deserialization failed")?;
        ros_publisher
            .publish(deserialized_message)
            .map_err(|_| Error::Ros)
            .wrap_err("failed to publish message via ROS")?;
    }

    bail!("no more available messages from Zenoh")
}

pub struct IdentityAdapter;

impl DeserializerAdapter<Vec<u8>> for IdentityAdapter {
    type Error = Error;

    type Decoded = Vec<u8>;

    fn supported_encodings() -> &'static [ros2_client::rustdds::RepresentationIdentifier] {
        &[
            RepresentationIdentifier::CDR_BE,
            RepresentationIdentifier::CDR_LE,
            RepresentationIdentifier::PL_CDR_LE,
        ]
    }

    fn transform_decoded(decoded: Self::Decoded) -> Vec<u8> {
        decoded
    }
}

impl DefaultDecoder<Vec<u8>> for IdentityAdapter {
    type Decoder = IdentityDecoder;

    const DECODER: Self::Decoder = IdentityDecoder;
}

#[derive(Clone)]
pub struct IdentityDecoder;

impl ros2_client::rustdds::no_key::Decode<Vec<u8>> for IdentityDecoder {
    type Error = Error;

    fn decode_bytes(
        self,
        input_bytes: &[u8],
        encoding: ros2_client::rustdds::RepresentationIdentifier,
    ) -> std::result::Result<Vec<u8>, Self::Error> {
        // Ok(input_bytes.to_vec())
        Ok(encoding
            .to_bytes()
            .into_iter()
            .chain(input_bytes.iter().copied())
            .collect())
    }
}

pub async fn forward_ros_to_zenoh(
    reader: DataReader<Vec<u8>, IdentityAdapter>,
    zenoh_session: Session,
    zenoh_topic_name: &'static str,
) -> Result<()> {
    let stream = reader.async_sample_stream();
    pin_mut!(stream);

    let zenoh_publisher = zenoh_session
        .declare_publisher(format!("booster/{zenoh_topic_name}"))
        .await
        .map_err(Error::Zenoh)
        .wrap_err("failed to create Zenoh publisher")?;

    while let Some(result) = stream.next().await {
        let serialized_message = result.wrap_err("read error occurred")?.into_value();

        // if !zenoh_topic_name.contains("state") {
        println!(
            "forwarding {}\n  bytes: {}",
            zenoh_topic_name,
            serialized_message.len()
        );
        if serialized_message.len() >= 32 {
            dbg!(&serialized_message[..32]);
        }
        // }

        zenoh_publisher
            .put(serialized_message)
            .await
            .map_err(Error::Zenoh)
            .wrap_err("failed to publish message via Zenoh")?;
    }

    bail!("no more available messages from ROS")
}
