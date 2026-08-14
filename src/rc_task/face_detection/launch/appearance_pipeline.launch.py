from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from face_detection.paths import default_guest_db_path

LAUNCH_DEFAULTS = {
    # Pipeline switches
    'personal_test_mode': 'false',
    'enable_profile_capture': 'true',
    'enable_identification': 'true',

    # External camera topic settings
    'image_topic': '/camera/camera/color/image_raw',
    'enable_depth': 'true',
    'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
    'depth_sync_slop_sec': '0.08',
    'face_depth_roi_radius_px': '4',

    # Optional external camera status input
    'stream_info_topic': '/camera/stream_info',

    # Perception topics / thresholds
    'face_topic': '/perception/largest_face',
    'appearance_topic': '/appearance/current_estimate',
    'recognized_identity_topic': '/appearance/recognized_human_identity',
    'recognized_identity_max_age_sec': '2.0',
    'debug_topic': '/perception/appearance',
    'det_score_threshold': '0.8',
    'yunet_model_path': '',
    'sface_model_path': '',
    'face_model_dir': '',
    'sface_model_name': 'face_recognition_sface_2021dec.onnx',
    'yunet_score_threshold': '0.8',
    'yunet_nms_threshold': '0.3',
    'yunet_top_k': '5000',
    'opencv_dnn_backend': 'default',
    'opencv_dnn_target': 'cpu',
    'face_inference_rate_hz': '5.0',
    'recognition_rate_hz': '2.0',
    'face_selection_mode': 'largest',
    'tracked_person_topic': '/human_identity/recognized_people',
    'target_track_id': '0',
    'tracked_roi_max_age_sec': '0.5',
    'max_face_age_sec': '0.5',
    'min_enrollment_samples': '5',
    'max_enrollment_samples': '10',
    'profile_capture_timeout_sec': '8.0',
    'recognition_threshold': '0.45',
    'recognition_margin': '0.05',
    'identity_window_size': '5',
    'identity_min_votes': '3',
    'recognition_action_timeout_sec': '3.0',
    'recognition_action_min_votes': '3',
    'debug_mode': 'false',
    'enable_inference_timing': 'false',
    'yolo_conf_threshold': '0.4',
    'appearance_device': '0',
    'enable_detected_items_node': 'false',
    'enable_stable_item_memory': 'false',
    'detected_items_topic': '/perception/detected_items',
    'detected_items_debug_topic': '/perception/detected_items_image',
    'publish_detected_items_debug_image': 'false',
    'stable_items_topic': '/perception/recognized_items',
    'get_visible_items_service': '/perception/get_visible_items',
    'stable_memory_min_confidence': '0.50',
    'stable_memory_window_size': '5',
    'stable_memory_min_votes': '3',
    'stable_memory_stale_timeout_sec': '1.0',
    'stable_memory_publish_rate_hz': '5.0',
    'show_appearance_gui': 'false',
    'show_stream_info_overlay': 'false',
    'appearance_window_name': 'Receptionist Appearance',
}


LAUNCH_DESCRIPTIONS = {
    'personal_test_mode': (
        'If true, consume /camera/image_raw and disable depth for an externally '
        'launched RGB camera.'
    ),
    'enable_profile_capture': 'If true, start the action server that saves named guest profiles.',
    'enable_identification': 'If true, start the node that matches visible guests against saved profiles.',
    'image_topic': 'External color image topic consumed by face_perception and appearance_node.',
    'enable_depth': 'If true, subscribe to synchronized depth for face distance estimation.',
    'depth_topic': 'External aligned depth topic consumed by face_perception.',
    'depth_sync_slop_sec': 'Maximum allowed timestamp difference when syncing color and depth frames.',
    'face_depth_roi_radius_px': 'Half-size in pixels of the face-centered depth sampling window.',
    'stream_info_topic': 'Optional external camera stream-status topic consumed by the appearance GUI.',
    'face_topic': 'Topic where the largest detected face estimate is published.',
    'appearance_topic': 'Topic where the current guest appearance estimate is published.',
    'recognized_identity_topic': (
        'Dedicated JSON topic carrying the temporally verified human identity for GUI debugging.'
    ),
    'recognized_identity_max_age_sec': (
        'Maximum identity-topic silence before the appearance GUI displays unavailable.'
    ),
    'debug_topic': 'Topic where appearance debug images or debug data are published.',
    'det_score_threshold': 'Minimum YuNet score accepted by enrollment.',
    'yunet_model_path': 'Explicit YuNet ONNX path; overrides package and external model directories.',
    'sface_model_path': 'Explicit SFace ONNX path; overrides package and external model directories.',
    'face_model_dir': 'External directory searched after the package models directory.',
    'sface_model_name': 'SFace filename used to bind persisted identity metadata.',
    'yunet_score_threshold': 'YuNet score threshold.',
    'yunet_nms_threshold': 'YuNet non-maximum suppression threshold.',
    'yunet_top_k': 'Maximum YuNet candidates retained before NMS.',
    'opencv_dnn_backend': 'OpenCV DNN backend: default or opencv.',
    'opencv_dnn_target': 'OpenCV DNN target; competition default and supported value is cpu.',
    'face_inference_rate_hz': 'Maximum rate for complete detection and recognition inference.',
    'recognition_rate_hz': 'Reserved lower-rate stable-identity recognition policy.',
    'face_selection_mode': 'Face selection mode: largest, target_roi, or tracked_person.',
    'tracked_person_topic': 'Existing vision/RecognizedPerson topic used for tracked-person ROI mode.',
    'target_track_id': 'Tracked person ID to select; zero selects the tracker nearest person.',
    'tracked_roi_max_age_sec': 'Maximum color-frame to tracked-ROI timestamp difference.',
    'max_face_age_sec': 'Maximum accepted age for enrollment and appearance face observations.',
    'min_enrollment_samples': 'Distinct consistent frames required for enrollment.',
    'max_enrollment_samples': 'Maximum enrollment embeddings retained before template selection.',
    'profile_capture_timeout_sec': 'Hard timeout for profile capture action goals.',
    'recognition_threshold': 'Minimum top-one cosine score for a known identity.',
    'recognition_margin': 'Minimum top-one minus top-two cosine score.',
    'identity_window_size': 'Sliding frame window used for continuous identity voting.',
    'identity_min_votes': 'Votes required to publish a stable continuous identity.',
    'recognition_action_timeout_sec': 'Time an action waits for new temporal face evidence.',
    'recognition_action_min_votes': 'New-frame votes required by a recognition action.',
    'debug_mode': 'Master face-pipeline debug switch; disabled for competition.',
    'enable_inference_timing': 'If true, periodically log YuNet/SFace timing metrics.',
    'yolo_conf_threshold': 'Minimum YOLO confidence score required to accept clothing or item detections.',
    'appearance_device': 'Ultralytics human-attribute device; defaults to CUDA GPU 0.',
    'enable_detected_items_node': 'If true, run the optional JSON detected-items YOLO node.',
    'enable_stable_item_memory': 'If true, publish multi-frame stable item names from detected-items JSON.',
    'detected_items_topic': 'Topic where per-frame detected item JSON messages are published.',
    'detected_items_debug_topic': 'Topic where detected item debug images are published.',
    'publish_detected_items_debug_image': 'If true, publish annotated debug images for detected items.',
    'stable_items_topic': 'Topic where temporally stable recognized item names are published.',
    'get_visible_items_service': 'Service name used to query currently visible stable items.',
    'stable_memory_min_confidence': 'Minimum item confidence required before stable memory considers a detection.',
    'stable_memory_window_size': 'Number of recent frames used by stable item memory voting.',
    'stable_memory_min_votes': 'Minimum votes inside the memory window required to publish an item as stable.',
    'stable_memory_stale_timeout_sec': 'Seconds before stable item memory drops detections that are no longer seen.',
    'stable_memory_publish_rate_hz': 'Rate in hertz for publishing stable recognized items.',
    'show_appearance_gui': 'If true, show the OpenCV appearance visualization window.',
    'show_stream_info_overlay': (
        'If true, draw camera resolution, refresh rate, source, and topic in '
        'the appearance OpenCV GUI.'
    ),
    'appearance_window_name': 'OpenCV window title used for the appearance visualization.',
    'db_path': 'Filesystem path for storing and loading receptionist guest profile data.',
}


def declare_launch_argument(name, default_value=None, description=''):
    if default_value is None:
        default_value = LAUNCH_DEFAULTS[name]
    if not description:
        description = LAUNCH_DESCRIPTIONS.get(name, '')
    return DeclareLaunchArgument(
        name,
        default_value=default_value,
        description=description,
    )
# This is important for this kind of default value declaration. 


def generate_launch_description():
    personal_test_mode = LaunchConfiguration('personal_test_mode')
    enable_profile_capture = LaunchConfiguration('enable_profile_capture')
    enable_identification = LaunchConfiguration('enable_identification')
    image_topic = LaunchConfiguration('image_topic')
    enable_depth = LaunchConfiguration('enable_depth')
    depth_topic = LaunchConfiguration('depth_topic')
    depth_sync_slop_sec = LaunchConfiguration('depth_sync_slop_sec')
    face_depth_roi_radius_px = LaunchConfiguration('face_depth_roi_radius_px')
    stream_info_topic = LaunchConfiguration('stream_info_topic')
    face_topic = LaunchConfiguration('face_topic')
    appearance_topic = LaunchConfiguration('appearance_topic')
    recognized_identity_topic = LaunchConfiguration('recognized_identity_topic')
    recognized_identity_max_age_sec = LaunchConfiguration('recognized_identity_max_age_sec')
    debug_topic = LaunchConfiguration('debug_topic')
    det_score_threshold = LaunchConfiguration('det_score_threshold')
    yunet_model_path = LaunchConfiguration('yunet_model_path')
    sface_model_path = LaunchConfiguration('sface_model_path')
    face_model_dir = LaunchConfiguration('face_model_dir')
    sface_model_name = LaunchConfiguration('sface_model_name')
    yunet_score_threshold = LaunchConfiguration('yunet_score_threshold')
    yunet_nms_threshold = LaunchConfiguration('yunet_nms_threshold')
    yunet_top_k = LaunchConfiguration('yunet_top_k')
    opencv_dnn_backend = LaunchConfiguration('opencv_dnn_backend')
    opencv_dnn_target = LaunchConfiguration('opencv_dnn_target')
    face_inference_rate_hz = LaunchConfiguration('face_inference_rate_hz')
    recognition_rate_hz = LaunchConfiguration('recognition_rate_hz')
    face_selection_mode = LaunchConfiguration('face_selection_mode')
    tracked_person_topic = LaunchConfiguration('tracked_person_topic')
    target_track_id = LaunchConfiguration('target_track_id')
    tracked_roi_max_age_sec = LaunchConfiguration('tracked_roi_max_age_sec')
    max_face_age_sec = LaunchConfiguration('max_face_age_sec')
    min_enrollment_samples = LaunchConfiguration('min_enrollment_samples')
    max_enrollment_samples = LaunchConfiguration('max_enrollment_samples')
    profile_capture_timeout_sec = LaunchConfiguration('profile_capture_timeout_sec')
    recognition_threshold = LaunchConfiguration('recognition_threshold')
    recognition_margin = LaunchConfiguration('recognition_margin')
    identity_window_size = LaunchConfiguration('identity_window_size')
    identity_min_votes = LaunchConfiguration('identity_min_votes')
    recognition_action_timeout_sec = LaunchConfiguration('recognition_action_timeout_sec')
    recognition_action_min_votes = LaunchConfiguration('recognition_action_min_votes')
    debug_mode = LaunchConfiguration('debug_mode')
    enable_inference_timing = LaunchConfiguration('enable_inference_timing')
    yolo_conf_threshold = LaunchConfiguration('yolo_conf_threshold')
    appearance_device = LaunchConfiguration('appearance_device')
    enable_detected_items_node = LaunchConfiguration('enable_detected_items_node')
    enable_stable_item_memory = LaunchConfiguration('enable_stable_item_memory')
    detected_items_topic = LaunchConfiguration('detected_items_topic')
    detected_items_debug_topic = LaunchConfiguration('detected_items_debug_topic')
    publish_detected_items_debug_image = LaunchConfiguration('publish_detected_items_debug_image')
    stable_items_topic = LaunchConfiguration('stable_items_topic')
    get_visible_items_service = LaunchConfiguration('get_visible_items_service')
    stable_memory_min_confidence = LaunchConfiguration('stable_memory_min_confidence')
    stable_memory_window_size = LaunchConfiguration('stable_memory_window_size')
    stable_memory_min_votes = LaunchConfiguration('stable_memory_min_votes')
    stable_memory_stale_timeout_sec = LaunchConfiguration('stable_memory_stale_timeout_sec')
    stable_memory_publish_rate_hz = LaunchConfiguration('stable_memory_publish_rate_hz')
    show_appearance_gui = LaunchConfiguration('show_appearance_gui')
    show_stream_info_overlay = LaunchConfiguration('show_stream_info_overlay')
    appearance_window_name = LaunchConfiguration('appearance_window_name')
    db_path = LaunchConfiguration('db_path')
    # Select external RGB-only topics for laptop testing when requested.
    # Keep the external RealSense topic defaults unless personal_test_mode:=true.
    effective_image_topic = PythonExpression([
        "'/camera/image_raw' if '", personal_test_mode, "' == 'true' else '", image_topic, "'"
    ])
    effective_enable_depth = PythonExpression([
        "'false' if '", personal_test_mode, "' == 'true' else '", enable_depth, "'"
    ])

    # Consume the externally published color and optional aligned-depth streams.
    face_perception_node = Node(
        package='face_detection',
        executable='face_perception',
        name='face_perception_node',
        output='screen',
        parameters=[
            {
                'image_topic': effective_image_topic,
                'enable_depth': effective_enable_depth,
                'depth_topic': depth_topic,
                'depth_sync_slop_sec': depth_sync_slop_sec,
                'depth_roi_radius_px': face_depth_roi_radius_px,
                'face_topic': face_topic,
                'yunet_model_path': yunet_model_path,
                'sface_model_path': sface_model_path,
                'face_model_dir': face_model_dir,
                'yunet_score_threshold': yunet_score_threshold,
                'yunet_nms_threshold': yunet_nms_threshold,
                'yunet_top_k': yunet_top_k,
                'opencv_dnn_backend': opencv_dnn_backend,
                'opencv_dnn_target': opencv_dnn_target,
                'face_inference_rate_hz': face_inference_rate_hz,
                'recognition_rate_hz': recognition_rate_hz,
                'max_frame_age_sec': max_face_age_sec,
                'face_selection_mode': face_selection_mode,
                'tracked_person_topic': tracked_person_topic,
                'target_track_id': target_track_id,
                'tracked_roi_max_age_sec': tracked_roi_max_age_sec,
                'debug_mode': debug_mode,
                'enable_inference_timing': enable_inference_timing,
            }
        ],
    )
    appearance_node = Node(
        package='face_detection',
        executable='appearance_receptionist',
        name='appearance_node',
        output='screen',
        parameters=[
            {
                'image_topic': effective_image_topic,
                'face_topic': face_topic,
                'appearance_topic': appearance_topic,
                'recognized_identity_topic': recognized_identity_topic,
                'recognized_identity_max_age_sec': recognized_identity_max_age_sec,
                'debug_topic': debug_topic,
                'stream_info_topic': stream_info_topic,
                'yolo_conf_threshold': yolo_conf_threshold,
                'appearance_device': ParameterValue(appearance_device, value_type=str),
                'max_face_age_sec': max_face_age_sec,
                'show_gui': show_appearance_gui,
                'show_stream_info_overlay': show_stream_info_overlay,
                'window_name': appearance_window_name,
            }
        ],
    )
    detected_items_node = Node(
        package='face_detection',
        executable='detected_items_node',
        name='detected_items_node',
        output='screen',
        condition=IfCondition(enable_detected_items_node),
        parameters=[
            {
                'image_topic': effective_image_topic,
                'detections_topic': detected_items_topic,
                'debug_image_topic': detected_items_debug_topic,
                'yolo_conf_threshold': yolo_conf_threshold,
                'publish_debug_image': publish_detected_items_debug_image,
            }
        ],
    )
    stable_item_memory_node = Node(
        package='face_detection',
        executable='stable_detected_items_node',
        name='stable_detected_items_node',
        output='screen',
        condition=IfCondition(enable_stable_item_memory),
        parameters=[
            {
                'input_detections_topic': detected_items_topic,
                'stable_items_topic': stable_items_topic,
                'get_visible_items_service': get_visible_items_service,
                'min_confidence': stable_memory_min_confidence,
                'window_size': stable_memory_window_size,
                'min_votes': stable_memory_min_votes,
                'stale_timeout_sec': stable_memory_stale_timeout_sec,
                'publish_rate_hz': stable_memory_publish_rate_hz,
            }
        ],
    )
    profile_capture_node = Node(
        package='face_detection',
        executable='profile_capture_receptionist',
        name='profile_capture_node',
        output='screen',
        condition=IfCondition(enable_profile_capture),
        parameters=[
            {
                'face_topic': face_topic,
                'appearance_topic': appearance_topic,
                'action_name': '/appearance/capture_named_profile',
                'db_path': db_path,
                'embedding_backend': 'opencv_sface',
                'embedding_model': sface_model_name,
                'embedding_dimension': 0,
                'min_enrollment_samples': min_enrollment_samples,
                'max_enrollment_samples': max_enrollment_samples,
                'max_face_age_sec': max_face_age_sec,
                'profile_capture_timeout_sec': profile_capture_timeout_sec,
                'det_score_threshold': det_score_threshold,
            }
        ],
    )
    profile_identification_node = Node(
        package='face_detection',
        executable='profile_identification_receptionist',
        name='profile_identification_node',
        output='screen',
        condition=IfCondition(enable_identification),
        parameters=[
            {
                'face_topic': face_topic,
                'appearance_topic': appearance_topic,
                'db_path': db_path,
                'embedding_backend': 'opencv_sface',
                'embedding_model': sface_model_name,
                'embedding_dimension': 0,
                'max_face_age_sec': max_face_age_sec,
                'recognition_threshold': recognition_threshold,
                'recognition_margin': recognition_margin,
                'identity_window_size': identity_window_size,
                'identity_min_votes': identity_min_votes,
                'recognition_action_timeout_sec': recognition_action_timeout_sec,
                'recognition_action_min_votes': recognition_action_min_votes,
                'recognized_identity_topic': recognized_identity_topic,
            }
        ],
    )
    guest_memory_node = Node(
        package='receptionist_nodes',
        executable='guest_memory_node',
        name='guest_memory_node',
        output='screen',
        condition=IfCondition(enable_profile_capture),
    )
    return LaunchDescription(
        [
            declare_launch_argument('personal_test_mode'),
            declare_launch_argument('enable_profile_capture'),
            declare_launch_argument('enable_identification'),
            declare_launch_argument('image_topic'),
            declare_launch_argument('enable_depth'),
            declare_launch_argument('depth_topic'),
            declare_launch_argument('depth_sync_slop_sec'),
            declare_launch_argument('face_depth_roi_radius_px'),
            declare_launch_argument('stream_info_topic'),
            declare_launch_argument('face_topic'),
            declare_launch_argument('appearance_topic'),
            declare_launch_argument('recognized_identity_topic'),
            declare_launch_argument('recognized_identity_max_age_sec'),
            declare_launch_argument('debug_topic'),
            declare_launch_argument('det_score_threshold'),
            declare_launch_argument('yunet_model_path'),
            declare_launch_argument('sface_model_path'),
            declare_launch_argument('face_model_dir'),
            declare_launch_argument('sface_model_name'),
            declare_launch_argument('yunet_score_threshold'),
            declare_launch_argument('yunet_nms_threshold'),
            declare_launch_argument('yunet_top_k'),
            declare_launch_argument('opencv_dnn_backend'),
            declare_launch_argument('opencv_dnn_target'),
            declare_launch_argument('face_inference_rate_hz'),
            declare_launch_argument('recognition_rate_hz'),
            declare_launch_argument('face_selection_mode'),
            declare_launch_argument('tracked_person_topic'),
            declare_launch_argument('target_track_id'),
            declare_launch_argument('tracked_roi_max_age_sec'),
            declare_launch_argument('max_face_age_sec'),
            declare_launch_argument('min_enrollment_samples'),
            declare_launch_argument('max_enrollment_samples'),
            declare_launch_argument('profile_capture_timeout_sec'),
            declare_launch_argument('recognition_threshold'),
            declare_launch_argument('recognition_margin'),
            declare_launch_argument('identity_window_size'),
            declare_launch_argument('identity_min_votes'),
            declare_launch_argument('recognition_action_timeout_sec'),
            declare_launch_argument('recognition_action_min_votes'),
            declare_launch_argument('debug_mode'),
            declare_launch_argument('enable_inference_timing'),
            declare_launch_argument('yolo_conf_threshold'),
            declare_launch_argument('appearance_device'),
            declare_launch_argument('enable_detected_items_node'),
            declare_launch_argument('enable_stable_item_memory'),
            declare_launch_argument('detected_items_topic'),
            declare_launch_argument('detected_items_debug_topic'),
            declare_launch_argument('publish_detected_items_debug_image'),
            declare_launch_argument('stable_items_topic'),
            declare_launch_argument('get_visible_items_service'),
            declare_launch_argument('stable_memory_min_confidence'),
            declare_launch_argument('stable_memory_window_size'),
            declare_launch_argument('stable_memory_min_votes'),
            declare_launch_argument('stable_memory_stale_timeout_sec'),
            declare_launch_argument('stable_memory_publish_rate_hz'),
            declare_launch_argument('show_appearance_gui'),
            declare_launch_argument('show_stream_info_overlay'),
            declare_launch_argument('appearance_window_name'),
            declare_launch_argument('db_path', default_guest_db_path()),
            face_perception_node,
            appearance_node,
            detected_items_node,
            stable_item_memory_node,
            profile_identification_node,
            guest_memory_node,
            profile_capture_node,
        ]
    )
