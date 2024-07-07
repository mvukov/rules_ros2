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

    let node = rclrs::create_node(&context, "minimal_subscriber")?;

    let _subscription = node.create_subscription::<chatter_interface::msg::Chatter, _>(
        "topic",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: rclrs::ReadOnlyLoanedMessage<'_, chatter_interface::msg::Chatter>| {
            let now = SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap()
                .as_micros() as u64;
            let delay_us = now - msg.timestamp;
            let data_length = msg.data_length as usize;
            println!(
                "Delay {} us, I heard: '{:?}'",
                delay_us,
                String::from_utf8(msg.data[..data_length].to_vec()).unwrap()
            );
        },
    )?;

    while !shut_down.load(Ordering::Relaxed) && context.ok() {
        match rclrs::spin_once(node.clone(), Some(std::time::Duration::from_millis(10))) {
            Err(rclrs::RclrsError::RclError {
                code: rclrs::RclReturnCode::Timeout,
                ..
            }) => (),
            Err(error) => {
                return Err(Box::new(error));
            }
            _ => (),
        };
    }
    Ok(())
}
