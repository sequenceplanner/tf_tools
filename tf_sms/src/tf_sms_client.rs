use r2r::geometry_msgs::msg::Transform;
use r2r::tf_tools_msgs::srv::ManipulateScene;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "tf_sms_client", "")?;

    let client = node.create_client::<ManipulateScene::Service>("manipulate_scene")?;
    let waiting = node.is_available(&client)?;

    let handle = tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    println!("waiting for service...");
    waiting.await?;
    println!("service available.");

    std::thread::sleep(std::time::Duration::from_millis(3000));

    let req = ManipulateScene::Request {
        command: "update".to_string(),
        parent_frame: "frame_4".to_string(),
        child_frame: "frame_2".to_string(),
        same_position_in_world: true,
        transform: Transform::default(),
    };
    if let Ok(resp) = client.request(&req)?.await {
        println!("{:?}", resp);
    }

    handle.await?;

    Ok(())
}
