#[cfg(test)]
mod test {
    use builtin_interfaces;

    #[test]
    fn test_time_default() {
        let _msg = builtin_interfaces::msg::Time::default();
    }

    #[test]
    fn test_serialization() {
        let msg = builtin_interfaces::msg::Time::default();
        let _json = serde_json::to_string(&msg).unwrap();
    }
}
