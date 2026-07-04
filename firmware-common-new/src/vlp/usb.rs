#[repr(u16)]
pub enum CliRequest {
    Invalid = 0,
    List = 1,
    Clear = 2,
    Download = 3,
}

impl Into<CliRequest> for u16 {
    fn into(self) -> CliRequest {
        match self {
            1 => CliRequest::List,
            2 => CliRequest::Clear,
            3 => CliRequest::Download,
            _ => CliRequest::Invalid,
        }
    }
}
