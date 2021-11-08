use futures::stream::Stream;
use futures::StreamExt;
use r2r::geometry_msgs::msg::Transform;
use r2r::geometry_msgs::msg::TransformStamped;
use r2r::tf_tools_msgs::srv::LookupTransform;
use r2r::tf_tools_msgs::srv::ManipulateBroadcast;
use r2r::tf_tools_msgs::srv::ManipulateScene;
use r2r::ServiceRequest;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "tf_sms", "")?;

    let server_requests = node.create_service::<ManipulateScene::Service>("manipulate_scene")?;
    let tf_lookup_client = node.create_client::<LookupTransform::Service>("tf_lookup")?;
    let tf_broadcast_client =
        node.create_client::<ManipulateBroadcast::Service>("manipulate_broadcast")?;

    let waiting_for_tf_lookup_server = node.is_available(&tf_lookup_client)?;
    let waiting_for_tf_broadcast_server = node.is_available(&tf_broadcast_client)?;

    let handle = std::thread::spawn(move || loop {
        &node.spin_once(std::time::Duration::from_millis(10));
    });

    r2r::log_warn!("sms", "Waiting for tf lookup service...");
    waiting_for_tf_lookup_server.await?;
    r2r::log_info!("sms", "TF lookup service available.");

    r2r::log_warn!("sms", "Waiting for tf broadcast service...");
    waiting_for_tf_broadcast_server.await?;
    r2r::log_info!("sms", "TF broadcast service available.");

    r2r::log_info!("sms", "TF Scene Manipulation Service Node started.");

    tokio::task::spawn(async move {
        let res = scene_manipulation_server(
            server_requests,
            tf_lookup_client,
            tf_broadcast_client,
        )
        .await;
        match res {
            Ok(()) => r2r::log_info!("sms", "Scene Manipulation Service call succeeded."),
            Err(e) => r2r::log_error!("sms", "Scene Manipulation Service call failed with: {}", e),
        };
    });

    handle.join().unwrap();

    Ok(())
}

async fn scene_manipulation_server(
    mut service: impl Stream<Item = ServiceRequest<ManipulateScene::Service>> + Unpin,
    tf_lookup_client: r2r::Client<LookupTransform::Service>,
    tf_broadcast_client: r2r::Client<ManipulateBroadcast::Service>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => match request.message.command.as_str() {
                "update" => {
                    r2r::log_info!("sms", "Got 'update' request: {:?}.", request.message);
                    let response =
                        update_frame(&request.message, &tf_lookup_client, &tf_broadcast_client)
                            .await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "remove" => {
                    r2r::log_info!("sms", "Got 'remove' request: {:?}.", request.message);
                    let response = remove_frame(&request.message, &tf_broadcast_client).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                _ => r2r::log_error!("sms", "No such command."),
            },
            None => {},
        }
    }
}

async fn update_frame(
    message: &r2r::tf_tools_msgs::srv::ManipulateScene::Request,
    tf_lookup_client: &r2r::Client<LookupTransform::Service>,
    tf_broadcast_client: &r2r::Client<ManipulateBroadcast::Service>,
) -> ManipulateScene::Response {
    match message.same_position_in_world {
        true => {
            match lookup_tf(
                &message.parent_frame,
                &message.child_frame,
                5000, //message.tf_lookup_deadline,
                &tf_lookup_client,
            )
            .await
            {
                Some(tf) => {
                    match manipulate_broadcast(
                        "update",
                        make_transform_stamped(
                            tf.transform.transform.clone(),
                            &message.child_frame,
                            &message.parent_frame,
                        ),
                        &tf_broadcast_client,
                    )
                    .await
                    {
                        Some(value) => ManipulateScene::Response { success: value },
                        None => ManipulateScene::Response { success: false },
                    }
                }
                None => ManipulateScene::Response { success: false },
            }
        }
        false => {
            match manipulate_broadcast(
                "update",
                make_transform_stamped(
                    message.transform.clone(),
                    &message.child_frame,
                    &message.parent_frame,
                ),
                &tf_broadcast_client,
            )
            .await
            {
                Some(value) => ManipulateScene::Response { success: value },
                None => ManipulateScene::Response { success: false },
            }
        }
    }
}

async fn remove_frame(
    message: &r2r::tf_tools_msgs::srv::ManipulateScene::Request,
    tf_broadcast_client: &r2r::Client<ManipulateBroadcast::Service>,
) -> ManipulateScene::Response {
    match manipulate_broadcast(
        "remove",
        make_transform_stamped(
            Transform::default(),
            &message.child_frame,
            &message.parent_frame,
        ),
        &tf_broadcast_client,
    )
    .await
    {
        Some(value) => ManipulateScene::Response { success: value },
        None => ManipulateScene::Response { success: false },
    }
}

fn make_transform_stamped(transform: Transform, child: &str, parent: &str) -> TransformStamped {
    let mut tf_stamped = TransformStamped::default();
    tf_stamped.transform = transform;
    tf_stamped.child_frame_id = child.to_string();
    tf_stamped.header.frame_id = parent.to_string();
    tf_stamped
}

async fn lookup_tf(
    child_id: &str,
    parent_id: &str,
    deadline: i32,
    client: &r2r::Client<LookupTransform::Service>,
) -> Option<LookupTransform::Response> {
    println!("sending request to tf lookup");

    let request = LookupTransform::Request {
        child_id: child_id.to_string(),
        parent_id: parent_id.to_string(),
        deadline: deadline,
    };

    let response = client
        .request(&request)
        .expect("could not send sms request")
        .await
        .expect("cancelled");

    println!("request to sms sent");

    Some(response)
}

async fn manipulate_broadcast(
    command: &str,
    transform: TransformStamped,
    client: &r2r::Client<ManipulateBroadcast::Service>,
) -> Option<bool> {
    println!("sending request to tf broadcast");

    let request = ManipulateBroadcast::Request {
        command: command.to_string(),
        transform: transform.clone(),
    };

    let response = client
        .request(&request)
        .expect("could not send manipulate broadcast request")
        .await
        .expect("cancelled");

    println!("request to sms sent");

    Some(response.success)
}