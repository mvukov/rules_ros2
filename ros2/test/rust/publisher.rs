use std::env;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use rclrs::{log_info, CreateBasicExecutor};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let shut_down = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shut_down))?;

    let context = rclrs::Context::new(env::args(), rclrs::InitOptions::default())?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("publisher")?;
    let publisher = node.create_publisher::<std_msgs::msg::String>("topic")?;

    let mut message = std_msgs::msg::String::default();
    let mut publish_count: u32 = 1;

    while !shut_down.load(Ordering::Relaxed) && context.ok() {
        let msg = format!("Hello, world! {}", publish_count);
        message.data = msg.clone();
        log_info!(node.logger(), "{}", msg);
        publisher.publish(&message)?;
        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    Ok(())
}
