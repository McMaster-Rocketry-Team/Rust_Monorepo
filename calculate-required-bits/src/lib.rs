use proc_macro::TokenStream;
use quote::quote;
use syn::parse::{Parse, ParseStream};
use syn::{parse_macro_input, Ident, LitFloat, Token};

struct Args {
    mode: String,
    min: f32,
    max: f32,
    max_error: f32,
}

impl Parse for Args {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mode: Ident = input.parse()?;
        input.parse::<Token![,]>()?;
        let min: LitFloat = input.parse()?;
        input.parse::<Token![,]>()?;
        let max: LitFloat = input.parse()?;
        input.parse::<Token![,]>()?;
        let max_error: LitFloat = input.parse()?;

        Ok(Args {
            mode: mode.to_string(),
            min: min.base10_parse()?,
            max: max.base10_parse()?,
            max_error: max_error.base10_parse()?,
        })
    }
}

#[proc_macro]
pub fn calculate_required_bits(input: TokenStream) -> TokenStream {
    let Args {
        mode,
        min,
        max,
        max_error,
    } = parse_macro_input!(input as Args);

    let expanded = if mode == "minmax" {
        let range = max - min;
        let bits = (range / max_error).log2().ceil() as usize;

        quote! {
            #bits
        }
    } else if mode == "slope" {
        let threshold_slope = min;
        let sample_time_ms = max;

        let range = threshold_slope * sample_time_ms / 1000.0 * 2.0;
        let bits = (range / max_error).log2().ceil() as usize;

        quote! {
            #bits
        }
    } else {
        panic!("Invalid mode, expected 'minmax' or 'slope'");
    };

    TokenStream::from(expanded)
}

#[proc_macro]
pub fn calculate_packed_type(input: TokenStream) -> TokenStream {
    let Args {
        mode,
        min,
        max,
        max_error,
    } = parse_macro_input!(input as Args);

    let bits = if mode == "minmax" {
        let range = max - min;
        (range / max_error).log2().ceil() as usize
    } else if mode == "slope" {
        let threshold_slope = min;
        let sample_time_ms = max;

        let range = threshold_slope * sample_time_ms / 1000.0 * 2.0;
        (range / max_error).log2().ceil() as usize
    } else {
        panic!("Invalid mode, expected 'minmax' or 'slope'");
    };

    let base_type =  match bits {
        1..=8 => quote! { u8 },
        9..=16 => quote! { u16 },
        17..=32 => quote! { u32 },
        33..=64 => quote! { u64 },
        _ => panic!("Bits out of range, expected 1 to 64"),
    };
    let expanded = quote! {
        packed_struct::prelude::Integer<#base_type, packed_struct::prelude::packed_bits::Bits<#bits>>
    };
    TokenStream::from(expanded)
}

#[proc_macro]
pub fn calculate_base_type(input: TokenStream) -> TokenStream {
    let Args {
        mode,
        min,
        max,
        max_error,
    } = parse_macro_input!(input as Args);

    let bits = if mode == "minmax" {
        let range = max - min;
        (range / max_error).log2().ceil() as usize
    } else if mode == "slope" {
        let threshold_slope = min;
        let sample_time_ms = max;

        let range = threshold_slope * sample_time_ms / 1000.0 * 2.0;
        (range / max_error).log2().ceil() as usize
    } else {
        panic!("Invalid mode, expected 'minmax' or 'slope'");
    };

    let base_type =  match bits {
        1..=8 => quote! { u8 },
        9..=16 => quote! { u16 },
        17..=32 => quote! { u32 },
        33..=64 => quote! { u64 },
        _ => panic!("Bits out of range, expected 1 to 64"),
    };
    TokenStream::from(base_type)
}

#[proc_macro]
pub fn calculate_min(input: TokenStream) -> TokenStream {
    let Args {
        mode,
        min,
        max,
        ..
    } = parse_macro_input!(input as Args);

    let expanded = if mode == "minmax" {
        quote! {
            #min
        }
    } else if mode == "slope" {
        let threshold_slope = min;
        let sample_time_ms = max;

        let min = -threshold_slope * sample_time_ms / 1000.0;
        quote! {
            #min
        }
    } else {
        panic!("Invalid mode, expected 'minmax' or 'slope'");
    };

    TokenStream::from(expanded)
}

#[proc_macro]
pub fn calculate_max(input: TokenStream) -> TokenStream {
    let Args {
        mode,
        min,
        max,
        ..
    } = parse_macro_input!(input as Args);

    let expanded = if mode == "minmax" {
        quote! {
            #max
        }
    } else if mode == "slope" {
        let threshold_slope = min;
        let sample_time_ms = max;

        let max = threshold_slope * sample_time_ms / 1000.0;
        quote! {
            #max
        }
    } else {
        panic!("Invalid mode, expected 'minmax' or 'slope'");
    };

    TokenStream::from(expanded)
}


struct DocStrArgs {
    mode: String,
    min: f32,
    max: f32,
    max_error: f32,
    name: Ident,
}

impl Parse for DocStrArgs {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mode: Ident = input.parse()?;
        input.parse::<Token![,]>()?;
        let min: LitFloat = input.parse()?;
        input.parse::<Token![,]>()?;
        let max: LitFloat = input.parse()?;
        input.parse::<Token![,]>()?;
        let max_error: LitFloat = input.parse()?;
        input.parse::<Token![,]>()?;
        let name: Ident = input.parse()?;

        Ok(DocStrArgs {
            mode: mode.to_string(),
            min: min.base10_parse()?,
            max: max.base10_parse()?,
            max_error: max_error.base10_parse()?,
            name,
        })
    }
}

#[proc_macro]
pub fn calculate_required_bits_docstr(input: TokenStream) -> TokenStream {
    let DocStrArgs {
        mode,
        min,
        max,
        max_error,
        name,
    } = parse_macro_input!(input as DocStrArgs);

    let expanded = if mode == "minmax" {
        let range = max - min;
        let bits = (range / max_error).log2().ceil() as usize;
        let actual_max_error = range / 2.0f32.powi(bits as i32);
        let docstr = format!(
            "Bits: {}, Range: {} ~ {}, Max Error: {}",
            bits, min, max, actual_max_error
        );
        let docstr = docstr.as_str();

        quote! {
            #[doc = #docstr]
            #[derive(Debug, Clone)]
            pub struct #name;
        }
    } else if mode == "slope" {
        let threshold_slope = min;
        let sample_time_ms = max;

        let min = -threshold_slope * sample_time_ms / 1000.0;
        let max = threshold_slope * sample_time_ms / 1000.0;
        let range = threshold_slope * sample_time_ms / 1000.0 * 2.0;
        let bits = (range / max_error).log2().ceil() as usize;
        let actual_max_error = range / 2.0f32.powi(bits as i32);
        let docstr = format!(
            "Threshold Slope: {} units / second, Sample Time: {}ms;\nBits: {}, Range: {} ~ {}, Max Error: {}",
            threshold_slope, sample_time_ms, bits, min, max, actual_max_error
        );
        let docstr = docstr.as_str();

        quote! {
            #[doc = #docstr]
            #[derive(Debug, Clone)]
            pub struct #name;
        }
    } else {
        panic!("Invalid mode, expected 'minmax' or 'slope'");
    };

    TokenStream::from(expanded)
}
