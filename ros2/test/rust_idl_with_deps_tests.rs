#[cfg(test)]
mod test {
    use sensor_msgs;

    #[test]
    fn test_image_default() {
        let _msg = sensor_msgs::msg::Image::default();
    }
}
