#[repr(u16)]
pub enum CliRequest {
    Invalid = 0,
    List = 1,
    Clear = 2,
    Download = 3,
}

impl From<u16> for CliRequest {
    fn from(value: u16) -> Self {
        match value {
            1 => CliRequest::List,
            2 => CliRequest::Clear,
            3 => CliRequest::Download,
            _ => CliRequest::Invalid,
        }
    }
}
