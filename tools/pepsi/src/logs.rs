use std::path::PathBuf;

use clap::Subcommand;
use color_eyre::{eyre::WrapErr, Result};

use nao::Nao;

use crate::{parsers::NaoAddress, progress_indicator::ProgressIndicator};

#[derive(Subcommand)]
pub enum Arguments {
    // Delete logs on the NAOs
    Delete {
        /// The NAOs to delete logs from e.g. 20w or 10.1.24.22
        #[arg(required = true)]
        naos: Vec<NaoAddress>,
    },
    // Download logs from the NAOs
    Download {
        /// Directory where to store the downloaded logs (will be created if not existing)
        log_directory: PathBuf,
        /// The NAOs to download logs from e.g. 20w or 10.1.24.22
        #[arg(required = true)]
        naos: Vec<NaoAddress>,
    },
}

pub async fn logs(arguments: Arguments) -> Result<()> {
    match arguments {
        Arguments::Delete { naos } => {
            ProgressIndicator::map_tasks(naos, "Deleting logs...", |nao_address| async move {
                let nao = Nao::new(nao_address.ip);
                nao.delete_logs()
                    .await
                    .wrap_err_with(|| format!("failed to delete logs on {nao_address}"))
            })
            .await
        }
        Arguments::Download {
            log_directory,
            naos,
        } => {
            ProgressIndicator::map_tasks(naos, "Downloading logs...", |nao_address| {
                let log_directory = log_directory.join(nao_address.to_string());
                async move {
                    let nao = Nao::new(nao_address.ip);
                    nao.download_logs(log_directory)
                        .await
                        .wrap_err_with(|| format!("failed to download logs from {nao_address}"))
                }
            })
            .await
        }
    }

    Ok(())
}
