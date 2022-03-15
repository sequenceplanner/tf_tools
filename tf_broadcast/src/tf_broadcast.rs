use r2r::geometry_msgs::msg::{Transform, TransformStamped};
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::ParameterValue;
use r2r::QosProfile;
use serde::{Deserialize, Serialize};
use std::fs::{self, File};
use std::io::BufReader;
use std::sync::{Arc, Mutex};

pub static NODE_ID: &'static str = "tf_broadcast";
pub static RELOAD_SCENE_RATE: u64 = 3000;
pub static STATIC_BROADCAST_RATE: u64 = 1000;
pub static ACTIVE_BROADCAST_RATE: u64 = 10;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct FrameData {
    pub parent_frame_id: String,
    pub child_frame_id: String,
    pub transform: Transform,
    pub active: bool,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // setup the node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    // handle parameters passed on from the launch files
    let params = node.params.clone();
    let params_things = params.lock().unwrap(); // OK to panic
    let scenario_path = params_things.get("scenario_path");

    let path_param = match scenario_path {
        Some(p) => match p {
            ParameterValue::String(path_param) => {
                path_param.clone()
            },
            _ => {
                r2r::log_warn!(
                    NODE_ID,
                    "Parameter 'scenario_path' has to be of type String. Empty scenario will be launched."
                );
                "".to_string()
            }
        },
        None => {
            r2r::log_warn!(
                NODE_ID,
                "Parameter 'scenario_path' not specified. Empty scenario will be launched."
            );
            "".to_string()
        }
    };

    let path = Arc::new(Mutex::new(path_param.clone()));
    let scenario = list_frames_in_dir(&path_param).await;

    // TODO: offer a service to change the scenario folder?

    let scenario_frames_init = load_scenario(&scenario).await;
    let scenario_frames = Arc::new(Mutex::new(scenario_frames_init));

    // publish the static frames to tf_static
    let static_pub_timer =
        node.create_wall_timer(std::time::Duration::from_millis(STATIC_BROADCAST_RATE))?;
    let static_frame_broadcaster = node.create_publisher::<TFMessage>(
        "tf_static",
        QosProfile::transient_local(QosProfile::default()),
    )?;

    // publish the active frames to tf
    let active_pub_timer =
        node.create_wall_timer(std::time::Duration::from_millis(ACTIVE_BROADCAST_RATE))?;
    let active_frame_broadcaster = node
        .create_publisher::<TFMessage>("tf", QosProfile::transient_local(QosProfile::default()))?;

    let reload_timer =
        node.create_wall_timer(std::time::Duration::from_millis(RELOAD_SCENE_RATE))?;

    // spawn a tokio task to handle publishing static frames
    let scenario_frames_clone_1 = scenario_frames.clone();
    tokio::task::spawn(async move {
        match static_frame_broadcaster_callback(
            static_frame_broadcaster,
            static_pub_timer,
            &scenario_frames_clone_1,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Static frame broadcaster failed with: '{}'.", e),
        };
    });

    // spawn a tokio task to handle publishing active frames
    let scenario_frames_clone_2 = scenario_frames.clone();
    tokio::task::spawn(async move {
        match active_frame_broadcaster_callback(
            active_frame_broadcaster,
            active_pub_timer,
            &scenario_frames_clone_2,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Active frame broadcaster failed with: '{}'.", e),
        };
    });

    // spawn a tokio task to handle reloading the scene when a new frame is to be added manually
    let scenario_frames_clone_3 = scenario_frames.clone();
    let path_clone_1 = path.clone();
    tokio::task::spawn(async move {
        match folder_loader_callback(
            reload_timer,
            &scenario_frames_clone_3,
            &path_clone_1,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Active frame broadcaster failed with: '{}'.", e),
        };
    });

    // keep the node alive
    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(1000));
    });

    r2r::log_info!(NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}

async fn list_frames_in_dir(path: &str) -> Vec<String> {
    let mut scenario = vec![];
    match fs::read_dir(path) {
        Ok(dir) => dir.for_each(|file| match file {
            Ok(entry) => match entry.path().to_str() {
                Some(valid) => scenario.push(valid.to_string()),
                None => r2r::log_warn!(NODE_ID, "Path is not valid unicode."),
            },
            Err(e) => r2r::log_warn!(NODE_ID, "Reading entry failed with '{}'.", e),
        }),
        Err(e) => {
            println!("{:?}", path);
            r2r::log_error!(
                NODE_ID,
                "Reading the scenario directory failed with: '{}'. Empty scenario will be launched.",
                e
            );
        }
    }
    scenario
}

async fn load_scenario(scenario: &Vec<String>) -> Vec<FrameData> {
    let mut frame_datas = vec![];
    scenario.iter().for_each(|x| match File::open(x) {
        Ok(file) => {
            let reader = BufReader::new(file);
            match serde_json::from_reader(reader) {
                Ok(json) => frame_datas.push(json),
                Err(e) => r2r::log_warn!(NODE_ID, "Serde failed with: '{}'.", e),
            }
        }
        Err(e) => r2r::log_warn!(NODE_ID, "Opening json file failed with: '{}'.", e),
    });
    frame_datas
}

// broadcast static frames
async fn static_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFMessage>,
    mut timer: r2r::Timer,
    transforms: &Arc<Mutex<Vec<FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = transforms.lock().unwrap().clone();
        let mut updated_transforms = vec![];

        transforms_local.iter().for_each(|t| match t.active {
            false => {
                updated_transforms.push(TransformStamped {
                    header: Header {
                        stamp: time_stamp.clone(),
                        frame_id: t.parent_frame_id.clone(),
                    },
                    child_frame_id: t.child_frame_id.clone(),
                    transform: t.transform.clone(),
                });
            }
            true => (),
        });

        let msg = TFMessage {
            transforms: updated_transforms,
        };

        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    NODE_ID,
                    "Static broadcaster failed to send a message with: '{}'",
                    e
                );
            }
        };
        timer.tick().await?;
    }
}

// broadcast active frames
async fn active_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFMessage>,
    mut timer: r2r::Timer,
    transforms: &Arc<Mutex<Vec<FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = transforms.lock().unwrap().clone();
        let mut updated_transforms = vec![];

        transforms_local.iter().for_each(|t| match t.active {
            true => {
                updated_transforms.push(TransformStamped {
                    header: Header {
                        stamp: time_stamp.clone(),
                        frame_id: t.parent_frame_id.clone(),
                    },
                    child_frame_id: t.child_frame_id.clone(),
                    transform: t.transform.clone(),
                });
            }
            false => (),
        });

        let msg = TFMessage {
            transforms: updated_transforms,
        };

        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    NODE_ID,
                    "Active broadcaster failed to send a message with: '{}'",
                    e
                );
            }
        };
        timer.tick().await?;
    }
}

// occasionally look at the scenario folder specified by the path and if there
// have been changes, add the the frames that have been manually added to the folder
async fn folder_loader_callback(
    mut timer: r2r::Timer,
    transforms: &Arc<Mutex<Vec<FrameData>>>,
    path: &Arc<Mutex<String>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let currently_loaded: Vec<String> = transforms
            .lock()
            .unwrap()
            .iter()
            .map(|x| x.child_frame_id.clone())
            .collect();
        let mut to_be_added = vec![];
        let path = path.lock().unwrap().clone();
        let frames = list_frames_in_dir(&path).await.clone();
        load_scenario(&frames).await.clone().iter().for_each(|x| {
            if !currently_loaded.contains(&x.child_frame_id.clone()) {
                to_be_added.push(x.clone())
            }
        });

        let mut transforms_local = transforms.lock().unwrap().clone();
        if !to_be_added.is_empty() {
            transforms_local.extend(to_be_added);
            *transforms.lock().unwrap() = transforms_local;
        }

        timer.tick().await?;
    }
}
