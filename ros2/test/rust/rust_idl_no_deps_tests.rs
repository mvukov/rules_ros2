#[cfg(test)]
mod test {
    use builtin_interfaces;

    #[test]
    fn test_time_default() {
        let _msg = builtin_interfaces::msg::Time::default();
    }
}
