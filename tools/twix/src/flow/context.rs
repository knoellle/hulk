use std::sync::Arc;

use crate::nao::Nao;

pub struct Context {
    pub robots: Vec<Arc<Nao>>,
}
