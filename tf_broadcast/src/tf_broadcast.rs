use r2r::geometry_msgs::msg::{Transform, TransformStamped};
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::ParameterValue;
use r2r::QosProfile;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::fs::{self, File};
use std::io::BufReader;
use std::sync::{Arc, Mutex};
use r2r::tf_tools_msgs::srv::ManipulateScene;
use r2r::tf_tools_msgs::srv::LookupTransform;

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

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ExtendedFrameData {
    pub frame_data: FrameData,
    pub folder_loaded: bool
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
            ParameterValue::String(path_param) => path_param.clone(),
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
    // TODO: offer a service to load new frames into memory (sms)
    // TODO: differentiate frames added through service or loaded from folder

    let init_loaded = load_scenario(&scenario).await;
    r2r::log_info!(
        NODE_ID,
        "Initial frames added to the scene: '{:?}'.",
        init_loaded.keys()
    );

    // differentiate frames loaded through service call or loaded from folder
    let frames = Arc::new(Mutex::new(init_loaded));
    // let service_frames = Arc::new(Mutex::new(HashMap::<String, FrameData>::new()));

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

    let scene_manipulation_service = node.create_service::<ManipulateScene::Service>("manipulate_scene")?;
    let tf_lookup_client = node.create_client::<LookupTransform::Service>("tf_lookup")?;
    let waiting_for_tf_lookup_service = node.is_available(&tf_lookup_client)?;

    // occasionally look into the folder specified by the path to see if there are changes
    let reload_timer =
        node.create_wall_timer(std::time::Duration::from_millis(RELOAD_SCENE_RATE))?;

    // spawn a tokio task to handle publishing static frames
    let frames_clone_1 = frames.clone();
    // let service_frames_clone_1 = service_frames.clone();
    tokio::task::spawn(async move {
        match static_frame_broadcaster_callback(
            static_frame_broadcaster,
            static_pub_timer,
            &frames_clone_1,
            // &service_frames_clone_1,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Static frame broadcaster failed with: '{}'.", e),
        };
    });

    // spawn a tokio task to handle publishing active frames
    let frames_clone_2 = frames.clone();
    // let service_frames_clone_2 = service_frames.clone();
    tokio::task::spawn(async move {
        match active_frame_broadcaster_callback(
            active_frame_broadcaster,
            active_pub_timer,
            &frames_clone_2,
            // &service_frames_clone_2,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Active frame broadcaster failed with: '{}'.", e),
        };
    });

    // spawn a tokio task to handle reloading the scene when a new frame is to be added manually
    let frames_clone_3 = frames.clone();
    let path_clone_1 = path.clone();
    tokio::task::spawn(async move {
        match folder_loader_callback(reload_timer, &frames_clone_3, &path_clone_1).await {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Active frame broadcaster failed with: '{}'.", e),
        };
    });

    // offer a service to manipulate the scene. This means adding (loading), updatind, and removing frames through a service call.
    // let remote_control_clone_2 = remote_control.clone();
    // tokio::task::spawn(async move {
    //     let result = remote_control_server(remote_control_service, &remote_control_clone_2).await;
    //     match result {
    //         Ok(()) => r2r::log_info!(NODE_ID, "Remote Control Service call succeeded."),
    //         Err(e) => r2r::log_error!(NODE_ID, "Remote Control Service call failed with: {}.", e),
    //     };
    // });

    // keep the node alive
    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(1000));
    });

    // before the other things in this node can start, it makes sense to wait for the tf lookup service to become alive
    r2r::log_warn!(NODE_ID, "Waiting for tf Lookup service...");
    waiting_for_tf_lookup_service.await?;
    r2r::log_info!(NODE_ID, "tf Lookup Service available.");

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

async fn load_scenario(scenario: &Vec<String>) -> HashMap<String, ExtendedFrameData> {
    let mut frame_datas: HashMap<String, ExtendedFrameData> = HashMap::new();
    scenario.iter().for_each(|x| match File::open(x) {
        Ok(file) => {
            let reader = BufReader::new(file);
            match serde_json::from_reader(reader) {
                Ok(jsonl) => {
                    let json = ExtendedFrameData {
                        frame_data: jsonl,
                        folder_loaded: true
                    };
                    frame_datas.insert(json.frame_data.child_frame_id.clone(), json.clone());
                }
                Err(e) => r2r::log_warn!(NODE_ID, "Serde failed with: '{}'.", e),
            }
        }
        Err(e) => r2r::log_warn!(NODE_ID, "Opening json file failed with: '{}'.", e),
    });
    frame_datas
}

// async fn load_scenario(scenario: &Vec<String>) -> Vec<FrameData> {
//     let mut frame_datas = vec![];
//     scenario.iter().for_each(|x| match File::open(x) {
//         Ok(file) => {
//             let reader = BufReader::new(file);
//             match serde_json::from_reader(reader) {
//                 Ok(json) => frame_datas.push(json),
//                 Err(e) => r2r::log_warn!(NODE_ID, "Serde failed with: '{}'.", e),
//             }
//         }
//         Err(e) => r2r::log_warn!(NODE_ID, "Opening json file failed with: '{}'.", e),
//     });
//     frame_datas
// }

// broadcast static frames
async fn static_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFMessage>,
    mut timer: r2r::Timer,
    frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
    // service_loaded: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = frames.lock().unwrap().clone();
        let mut updated_transforms = vec![];

        transforms_local.iter().for_each(|(_, v)| match v.frame_data.active {
            false => {
                updated_transforms.push(TransformStamped {
                    header: Header {
                        stamp: time_stamp.clone(),
                        frame_id: v.frame_data.parent_frame_id.clone(),
                    },
                    child_frame_id: v.frame_data.child_frame_id.clone(),
                    transform: v.frame_data.transform.clone(),
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
                    "Active broadcaster failed to send a message with: '{}'",
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
    frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
    // service_loaded: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = frames.lock().unwrap().clone();
        let mut updated_transforms = vec![];

        transforms_local.iter().for_each(|(_, v)| match v.frame_data.active {
            true => {
                updated_transforms.push(TransformStamped {
                    header: Header {
                        stamp: time_stamp.clone(),
                        frame_id: v.frame_data.parent_frame_id.clone(),
                    },
                    child_frame_id: v.frame_data.child_frame_id.clone(),
                    transform: v.frame_data.transform.clone(),
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
// have been changes, add or remove the the frames that have been manually added
// to the folder or removed from the folder
async fn folder_loader_callback(
    mut timer: r2r::Timer,
    frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
    path: &Arc<Mutex<String>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        // let mut to_be_added = vec![];
        let path = path.lock().unwrap().clone();
        let dir_frames = list_frames_in_dir(&path).await.clone();

        // differentiate between frames loaded from folder and through services
        // for example, if we add a frame through the service call, it will be 
        // removed every RELOAD_SCENE_RATE since it is not in the folder
        let mut folder_scenario = load_scenario(&dir_frames).await;
        folder_scenario.retain(|_, v| v.folder_loaded);
        let mut local_scenario = frames.lock().unwrap().clone();
        local_scenario.retain(|_, v| v.folder_loaded);

        let folder_vec = folder_scenario
            .iter()
            .map(|(k, _)| k.clone())
            .collect::<Vec<String>>();
        let loaded_vec = local_scenario
            .iter()
            .map(|(k, _)| k.clone())
            .collect::<Vec<String>>();

        let folder_set: HashSet<String> =
            HashSet::from_iter(folder_vec.clone().iter().map(|x| x.clone()));
        let local_set: HashSet<String> =
            HashSet::from_iter(loaded_vec.clone().iter().map(|x| x.clone()));

        let to_be_added: Vec<String> = folder_set
            .difference(&local_set)
            .map(|x| x.clone())
            .collect();
        let to_be_removed: Vec<String> = local_set
            .difference(&folder_set)
            .map(|x| x.clone())
            .collect();

        let mut transforms_local = frames.lock().unwrap().clone();

        to_be_added.iter().for_each(|x| {
            transforms_local.insert(x.clone(), folder_scenario.get(&*x).unwrap().clone());
            r2r::log_info!(NODE_ID, "Added frame: '{}'.", x);
        });

        to_be_removed.iter().for_each(|x| {
            transforms_local.remove(x);
            r2r::log_info!(NODE_ID, "Removed frame: '{}'.", x);
        });

        *frames.lock().unwrap() = transforms_local;

        timer.tick().await?;
    }
}
