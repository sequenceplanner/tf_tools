use futures::stream::Stream;
use futures::Future;
use futures::StreamExt;
use r2r::geometry_msgs::msg::Transform;
use r2r::geometry_msgs::msg::TransformStamped;
use r2r::tf_tools_msgs::srv::GetBroadcastedFrames;
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
    let get_broadcasted_frames_client =
        node.create_client::<GetBroadcastedFrames::Service>("get_broadcasted_frames")?;

    let waiting_for_tf_lookup_server = node.is_available(&tf_lookup_client)?;
    let waiting_for_tf_broadcast_server = node.is_available(&tf_broadcast_client)?;
    let waiting_for_get_broadcasted_frames_server =
        node.is_available(&get_broadcasted_frames_client)?;

    r2r::log_info!("log", "TF Scene Manipulation Service Node started");

    tokio::task::spawn(async move {
        let res = scene_manipulation_server(
            server_requests,
            tf_lookup_client,
            tf_broadcast_client,
            get_broadcasted_frames_client,
            waiting_for_tf_lookup_server,
            waiting_for_tf_broadcast_server,
            waiting_for_get_broadcasted_frames_server,
        )
        .await;
        match res {
            Ok(()) => r2r::log_info!("log", "Scene Manipulation Service call succeeded"),
            Err(e) => r2r::log_error!("log", "Scene Manipulation Service call failed with: {}", e),
        };
    });

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(10));
    });

    handle.join().unwrap();

    Ok(())
}

async fn scene_manipulation_server(
    mut service: impl Stream<Item = ServiceRequest<ManipulateScene::Service>> + Unpin,
    tf_lookup_client: r2r::Client<LookupTransform::Service>,
    tf_broadcast_client: r2r::Client<ManipulateBroadcast::Service>,
    get_broadcasted_frames_client: r2r::Client<GetBroadcastedFrames::Service>,
    waiting_for_tf_lookup_server: impl Future<Output = r2r::Result<()>>,
    waiting_for_tf_broadcast_server: impl Future<Output = r2r::Result<()>>,
    waiwaiting_for_get_broadcasted_frames_server: impl Future<Output = r2r::Result<()>>,
) -> Result<(), Box<dyn std::error::Error>> {
    r2r::log_warn!("log", "Waiting for tf lookup service...");
    waiting_for_tf_lookup_server.await?;
    r2r::log_info!("log", "TF lookup service available.");

    r2r::log_warn!("log", "Waiting for tf broadcast service...");
    waiting_for_tf_broadcast_server.await?;
    r2r::log_info!("log", "TF broadcast service available.");

    r2r::log_warn!("log", "Waiting for get broadcasted frames service...");
    waiwaiting_for_get_broadcasted_frames_server.await?;
    r2r::log_info!("log", "Get broadcasted frames service available.");

    loop {
        match service.next().await {
            Some(req) => match req.message.command.as_str() {
                "update" => {
                    let frames_result =
                        get_broadcasted_frames(&get_broadcasted_frames_client).await;
                    match frames_result {
                        Some(frames) => {
                            if frames.static_transforms.contains(&req.message.child_frame) {
                                r2r::log_warn!(
                                    "log",
                                    "Can't manipulate static frame {}.",
                                    req.message.child_frame
                                );
                            } else if frames.active_transforms.contains(&req.message.child_frame) {
                                if !req.message.same_position_in_world {
                                    manipulate_broadcast(
                                        "update",
                                        make_transform_stamped(
                                            req.message.transform.clone(),
                                            &req.message.child_frame,
                                            &req.message.parent_frame,
                                        ),
                                        &tf_broadcast_client,
                                    )
                                    .await;
                                    let resp = ManipulateScene::Response { success: true };
                                    req.respond(resp).expect("Could not send service response.");
                                    break;
                                } else {
                                    let tf_result = lookup_tf(
                                        &req.message.parent_frame,
                                        &req.message.child_frame,
                                        5000,
                                        &tf_lookup_client,
                                    )
                                    .await;
                                    match tf_result {
                                        Some(x) => {
                                            manipulate_broadcast(
                                                "update",
                                                make_transform_stamped(
                                                    x.transform.transform.clone(),
                                                    &req.message.child_frame,
                                                    &req.message.parent_frame,
                                                ),
                                                &tf_broadcast_client,
                                            )
                                            .await;
                                            let resp = ManipulateScene::Response { success: true };
                                            req.respond(resp)
                                                .expect("Could not send service response.");
                                            break;
                                        }
                                        None => {
                                            r2r::log_error!(
                                                "log",
                                                "Got None result from tf_lookup."
                                            );
                                            let resp = ManipulateScene::Response { success: false };
                                            req.respond(resp)
                                                .expect("Could not send service response.");
                                            break;
                                        }
                                    }
                                }
                            } else {
                                manipulate_broadcast(
                                    "add",
                                    make_transform_stamped(
                                        req.message.transform.clone(),
                                        &req.message.child_frame,
                                        &req.message.parent_frame,
                                    ),
                                    &tf_broadcast_client,
                                )
                                .await;
                                let resp = ManipulateScene::Response { success: true };
                                req.respond(resp).expect("could not send service response");
                                break;
                            }
                        }
                        None => r2r::log_error!(
                            "log",
                            "Got None result from get broadcasted frames."
                        )
                    }
                }
                "remove" => {
                    manipulate_broadcast(
                        "remove",
                        make_transform_stamped(
                            req.message.transform.clone(),
                            &req.message.child_frame,
                            &req.message.parent_frame,
                        ),
                        &tf_broadcast_client,
                    )
                    .await;
                    let resp = ManipulateScene::Response { success: true };
                    req.respond(resp).expect("could not send service response");
                    break;
                }
                _ => (),
            },
            None => break,
        }
    }

    Ok(())
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

async fn get_broadcasted_frames(
    client: &r2r::Client<GetBroadcastedFrames::Service>,
) -> Option<GetBroadcastedFrames::Response> {
    println!("sending request to tf get broadcasted frames");

    let request = GetBroadcastedFrames::Request {};

    let response = client
        .request(&request)
        .expect("could not send manipulate broadcast request")
        .await
        .expect("cancelled");

    println!("request to sms sent");

    Some(response)
}
