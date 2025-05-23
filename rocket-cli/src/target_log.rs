
#[derive(Debug, Clone)]
pub struct TargetLog {
    pub log_content: String,
    pub crate_name: String,
    pub file_name: String,
    pub file_path: String,
    pub line_number: String,
    pub log_level: String,
    pub module_path: String,
    pub timestamp: String,
}
