use r2r::geometry_msgs::msg::Transform;
use r2r::tf_tools_msgs::srv::ManipulateScene;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "tf_sms_client", "")?;

    let client = node.create_client::<ManipulateScene::Service>("manipulate_scene")?;
    let waiting = node.is_available(&client)?;

    let handle = std::thread::spawn(move || loop {
        &node.spin_once(std::time::Duration::from_millis(100));
    });

    r2r::log_warn!("sms_test", "Waiting for TF Scene Manipulation Service...");
    waiting.await?;
    r2r::log_info!("sms_test", "TF Scene Manipulation Service available.");
    r2r::log_info!(
        "sms_test",
        "TF Scene Manipulation Service Test Node started."
    );

    std::thread::sleep(std::time::Duration::from_millis(3000));

    let mut messages = vec![];

    let message_1 = ManipulateScene::Request {
        command: "update".to_string(),
        parent_frame: "frame_4".to_string(),
        child_frame: "frame_2".to_string(),
        same_position_in_world: true,
        transform: Transform::default(),
    };
    messages.push(message_1);

    let message_2 = ManipulateScene::Request {
        command: "update".to_string(),
        parent_frame: "world".to_string(),
        child_frame: "frame_4".to_string(),
        same_position_in_world: false,
        transform: Transform::default(),
    };
    messages.push(message_2);

    let message_3 = ManipulateScene::Request {
        command: "remove".to_string(),
        parent_frame: "world".to_string(),
        child_frame: "frame_4".to_string(),
        same_position_in_world: false,
        transform: Transform::default(),
    };
    messages.push(message_3);

    for message in messages {
        sms_test(
            &client,
            message
        ).await?;
    }
    
    handle.join().unwrap();

    Ok(())
}

async fn sms_test(
    client: &r2r::Client<ManipulateScene::Service>,
    message: ManipulateScene::Request,
) -> Result<(), Box<dyn std::error::Error>> {

    let response = client
        .request(&message)
        .expect("Could not send SMS request.")
        .await
        .expect("Cancelled.");

    r2r::log_info!("sms_test", "Request to SMS sent.");

    match response.success {
        true => {
            r2r::log_info!("sms_test", "Got sms response: {}", response.success);
        }
        false => {
            r2r::log_error!(
                "sms_test",
                "Couldn't manipulate scene for command command '{}'.",
                message.command
            );
        }
    }

    std::thread::sleep(std::time::Duration::from_millis(3000));

    Ok(())
}
