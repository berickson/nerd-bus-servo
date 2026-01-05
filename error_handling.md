# Error Handling

Commands return `bool` (true = success), queries return `std::optional<T>` (has value on success). Use `ok()` to check if the last operation succeeded, or `last_error()` for detailed diagnostics.
