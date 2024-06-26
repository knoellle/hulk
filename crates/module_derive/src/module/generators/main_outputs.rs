use module_attributes::{MainOutputAttribute, ModuleInformation};
use proc_macro2::TokenStream;
use quote::quote;
use syn::Ident;

fn generate_main_outputs_field(attribute: &MainOutputAttribute) -> TokenStream {
    let name = &attribute.name;
    let data_type = &attribute.data_type;
    quote! { #name: Option<#data_type> }
}

pub fn generate_main_outputs_struct(cycle_method: &ModuleInformation) -> TokenStream {
    let fields = cycle_method
        .main_outputs
        .iter()
        .map(generate_main_outputs_field);
    let main_outputs_identifier = &cycle_method.main_outputs_identifier;
    quote! {
        #[derive(Default)]
        struct #main_outputs_identifier {
            #(#fields),*
        }
    }
}

fn generate_main_outputs_assignment(attribute: &MainOutputAttribute) -> TokenStream {
    let name = &attribute.name;
    quote! {
        database.main_outputs.#name = match &injected_outputs.main_outputs.#name {
            Some(injected_output) => Some(injected_output.clone()),
            None => self.#name
        }
    }
}

pub fn generate_main_outputs_implementation(
    cycler: &Ident,
    cycle_method: &ModuleInformation,
) -> TokenStream {
    let assignments = cycle_method
        .main_outputs
        .iter()
        .map(generate_main_outputs_assignment);
    let main_outputs_identifier = &cycle_method.main_outputs_identifier;
    quote! {
        impl #main_outputs_identifier {
            fn update(
                self,
                database: &mut crate::#cycler::Database,
                injected_outputs: &crate::#cycler::Database,
            ) {
                #(#assignments);*
            }
            fn none() -> Self {
                Default::default()
            }
        }
    }
}
