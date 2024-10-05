use std::env;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::SystemTime;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let shut_down = Arc::new(AtomicBool::new(false));
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        signal_hook::flag::register(signal, Arc::clone(&shut_down))?;
    }

    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "minimal_publisher")?;
    let callback_period_ms = node
        .declare_parameter::<i64>("callback_period_ms")
        .default(500)
        .read_only()
        .unwrap()
        .get();

    let publisher = node.create_publisher::<chatter_interface::msg::rmw::Chatter>(
        "topic",
        rclrs::QOS_PROFILE_DEFAULT,
    )?;

    let mut publish_count: u32 = 0;

    while !shut_down.load(Ordering::Relaxed) && context.ok() {
        let mut message = publisher.borrow_loaned_message()?;
        message.timestamp = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)?
            .as_micros() as u64;
        message.count = publish_count as u64;

        let msg = format!("Hello, world! {publish_count}");
        let msg_len = msg.len();
        message.data_length = msg_len as u64;
        message.data[..msg_len].copy_from_slice(msg.as_bytes());
        rclrs::log_info!(node.logger_name(), "Publishing: {}", msg);
        message.publish()?;
        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(callback_period_ms as u64));
    }
    Ok(())
}
