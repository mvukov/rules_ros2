use std::env;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let shut_down = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shut_down))?;

    let context = rclrs::Context::new(env::args())?;
    let node = rclrs::create_node(&context, "publisher")?;
    let publisher =
        node.create_publisher::<std_msgs::msg::String>("topic", rclrs::QOS_PROFILE_DEFAULT)?;

    let mut message = std_msgs::msg::String::default();
    let mut publish_count: u32 = 1;

    while !shut_down.load(Ordering::Relaxed) && context.ok() {
        let msg = format!("Hello, world! {}", publish_count);
        message.data = msg.clone();
        rclrs::log_info!(node.logger_name(), "{}", msg);
        publisher.publish(&message)?;
        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    Ok(())
}
