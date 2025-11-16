use std::env;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::SystemTime;

use rclrs::{log_info, CreateBasicExecutor, IntoPrimitiveOptions, RclrsErrorFilter};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let shut_down = Arc::new(AtomicBool::new(false));
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        signal_hook::flag::register(signal, Arc::clone(&shut_down))?;
    }

    let context = rclrs::Context::new(env::args(), rclrs::InitOptions::default())?;
    let mut executor = context.create_basic_executor();

    let node = executor.create_node("minimal_subscriber")?;
    let subscription_node = node.clone();
    let _subscription = node.create_subscription(
        "topic".keep_last(1).reliable(),
        move |msg: rclrs::ReadOnlyLoanedMessage<chatter_interface::msg::Chatter>| {
            let now = SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap()
                .as_micros() as u64;
            let delay_us = now - msg.timestamp;
            let data_length = msg.data_length as usize;
            log_info!(
                subscription_node.logger(),
                "Delay {} us, I heard: '{:?}'",
                delay_us,
                String::from_utf8(msg.data[..data_length].to_vec()).unwrap()
            );
        },
    )?;

    while !shut_down.load(Ordering::Relaxed) && context.ok() {
        match executor
            .spin(rclrs::SpinOptions::spin_once().timeout(std::time::Duration::from_millis(10)))
            .first_error()
        {
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
