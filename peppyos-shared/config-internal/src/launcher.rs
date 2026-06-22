mod types;

// Only the validated `Name` identifier survives; the launcher document parser
// is daemon-only and not part of this library.
pub use types::Name;
