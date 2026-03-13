#[cfg(test)]
mod test {
    use rclrs::log_info;

    #[test]
    fn test_logging() {
        log_info!("test123", "test456");
    }
}
