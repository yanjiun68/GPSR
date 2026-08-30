#!/usr/bin/env python3
"""YASMIN state machine that speaks a stable InsightFace appearance summary."""

# Section: Standard-library imports support JSON parsing, process supervision, and voting.
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from collections import defaultdict

# Section: ROS 2 and YASMIN imports provide messaging, services, and state execution.
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from receptionist_interfaces.msg import GuestAppearance
from std_msgs.msg import String
from voice_interfaces.srv import SayText
from yasmin import Blackboard, State, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED
from yasmin_viewer import YasminViewerPub


# Section: Shared outcomes and registry paths define the machine and cleanup contract.
FAILED = 'failed'
REGISTRY_DIR = Path('/tmp/insightface_feature_summary_smacc2')
INSIGHTFACE_PGID_FILE = REGISTRY_DIR / 'active_insightface_pgid'


# Special function: Return a Blackboard value while supporting both YASMIN and test mappings.
def blackboard_get(blackboard, key, default):
    """Read a key without requiring it to have been initialized first."""
    # Section: Native YASMIN Blackboard exposes contains; dictionaries use membership.
    if hasattr(blackboard, 'contains'):
        return blackboard[key] if blackboard.contains(key) else default
    return blackboard[key] if key in blackboard else default


# Special function: Normalize a color before it participates in voting or speech.
def normalize_color(color):
    """Return a lowercase usable color or an empty string for unknown values."""
    # Section: Empty and sentinel values must not win the stable-color vote.
    normalized = str(color).strip().lower()
    return '' if normalized in ('', 'unknown', 'none', 'n/a') else normalized


# Special function: Choose the stable YES, NO, or UNKNOWN value for one attribute.
def choose_attribute_state(samples, state_field, confidence_field):
    """Vote on an appearance attribute and retain the winning maximum confidence."""
    # Section: Count semantic states and separately retain their highest confidences.
    yes_count = 0
    no_count = 0
    yes_confidence = 0.0
    no_confidence = 0.0
    for sample in samples:
        state = int(getattr(sample, state_field))
        confidence = float(getattr(sample, confidence_field))
        if state == GuestAppearance.YES:
            yes_count += 1
            yes_confidence = max(yes_confidence, confidence)
        elif state == GuestAppearance.NO:
            no_count += 1
            no_confidence = max(no_confidence, confidence)

    # Section: A YES vote wins ties to preserve the original SMACC2 behavior exactly.
    if yes_count > 0 and yes_count >= no_count:
        return int(GuestAppearance.YES), yes_confidence
    if no_count > 0 and no_count > yes_count:
        return int(GuestAppearance.NO), no_confidence
    return int(GuestAppearance.UNKNOWN), 0.0


# Special function: Aggregate several observations into stable appearance features.
def aggregate_appearance(samples):
    """Return the voted color and binary attributes from collected messages."""
    # Section: Color scores track occurrence count and summed confidence per normalized color.
    color_scores = defaultdict(lambda: [0, 0.0])
    for sample in samples:
        color = normalize_color(sample.shirt_color)
        if color:
            color_scores[color][0] += 1
            color_scores[color][1] += float(sample.shirt_color_confidence)

    # Section: Prefer count, then average confidence, then deterministic lexical ordering.
    shirt_color = ''
    shirt_color_confidence = 0.0
    if color_scores:
        shirt_color = max(
            sorted(color_scores),
            key=lambda color: (
                color_scores[color][0],
                color_scores[color][1] / color_scores[color][0],
            ),
        )
        count, confidence_sum = color_scores[shirt_color]
        shirt_color_confidence = confidence_sum / count

    # Section: Each semantic attribute is voted independently.
    glasses_state, glasses_confidence = choose_attribute_state(
        samples, 'glasses', 'glasses_confidence'
    )
    hat_state, hat_confidence = choose_attribute_state(samples, 'hat', 'hat_confidence')
    face_mask_state, face_mask_confidence = choose_attribute_state(
        samples, 'face_mask', 'face_mask_confidence'
    )
    return {
        'summary_sample_count': len(samples),
        'shirt_color': shirt_color,
        'shirt_color_confidence': shirt_color_confidence,
        'glasses_state': glasses_state,
        'glasses_confidence': glasses_confidence,
        'hat_state': hat_state,
        'hat_confidence': hat_confidence,
        'face_mask_state': face_mask_state,
        'face_mask_confidence': face_mask_confidence,
    }


# Special function: Render one semantic attribute as a complete speech sentence.
def describe_attribute(state, yes_sentence, no_sentence, unknown_sentence):
    """Translate the GuestAppearance enum into natural-language text."""
    # Section: Preserve UNKNOWN explicitly instead of treating it as a negative detection.
    if state == GuestAppearance.YES:
        return yes_sentence
    if state == GuestAppearance.NO:
        return no_sentence
    return unknown_sentence


# Special function: Build the final flexible summary from stable Blackboard values.
def build_feature_summary_text(shirt_color, glasses_state, hat_state, face_mask_state):
    """Create the single text request sent to the speech service."""
    # Section: Shirt color starts the summary and reports uncertainty honestly.
    if shirt_color:
        text = f'I can see a person wearing a {shirt_color} shirt. '
    else:
        text = 'I can see a person, but I cannot confidently determine the shirt color. '

    # Section: Append independent descriptions for glasses, hat, and face mask.
    text += describe_attribute(
        glasses_state,
        'I detect glasses. ',
        'I do not detect glasses. ',
        'I cannot confidently determine whether the person is wearing glasses. ',
    )
    text += describe_attribute(
        hat_state,
        'I detect a hat. ',
        'I do not detect a hat. ',
        'I cannot confidently determine whether the person is wearing a hat. ',
    )
    text += describe_attribute(
        face_mask_state,
        'I detect a face mask.',
        'I do not detect a face mask.',
        'I cannot confidently determine whether the person is wearing a face mask.',
    )
    return text


# Special function: Check whether a registered POSIX process group still exists.
def process_group_running(process_group_id, process=None):
    """Return True when the process group exists or cannot be signaled by this user."""
    # Section: Poll a known direct child first so a stopped leader cannot remain as a zombie.
    if process_group_id <= 0:
        return False
    if process is not None:
        process.poll()
    try:
        os.killpg(process_group_id, 0)
        return True
    except PermissionError:
        return True
    except ProcessLookupError:
        return False


# Special function: Save the detector process group for lifecycle-wide cleanup.
def write_insightface_pgid(process_group_id):
    """Persist the active process group atomically enough for the lifecycle wrapper."""
    # Section: The registry is shared with the existing C++ lifecycle supervisor.
    REGISTRY_DIR.mkdir(parents=True, exist_ok=True)
    INSIGHTFACE_PGID_FILE.write_text(f'{process_group_id}\n', encoding='utf-8')


# Special function: Read the registered process group if a prior run created one.
def read_insightface_pgid():
    """Return the registered process group or -1 for missing or malformed state."""
    # Section: Registry corruption must never prevent a new state-machine run.
    try:
        return int(INSIGHTFACE_PGID_FILE.read_text(encoding='utf-8').strip())
    except (FileNotFoundError, OSError, ValueError):
        return -1


# Special function: Stop a process group using bounded INT, TERM, and KILL escalation.
def stop_process_group(process_group_id, logger, process=None):
    """Stop the full ROS launch tree and return whether it exited."""
    # Section: An absent process is already clean.
    if not process_group_running(process_group_id, process):
        return True
    for signal_number, signal_name, wait_seconds in (
        (signal.SIGINT, 'SIGINT', 3.0),
        (signal.SIGTERM, 'SIGTERM', 2.0),
        (signal.SIGKILL, 'SIGKILL', 1.0),
    ):
        logger.info(f'Stopping InsightFace process group {process_group_id} with {signal_name}.')
        try:
            os.killpg(process_group_id, signal_number)
        except ProcessLookupError:
            return True
        deadline = time.monotonic() + wait_seconds
        while time.monotonic() < deadline:
            if not process_group_running(process_group_id, process):
                return True
            time.sleep(0.1)
    logger.error(f'InsightFace process group {process_group_id} did not stop.')
    return False


# Special function: Stop the registered launch tree and clear stale registry data.
def stop_registered_insightface(logger, process=None):
    """Clean up InsightFace regardless of which state initiated shutdown."""
    # Section: Clear the registry only after the process group is confirmed stopped.
    stopped = stop_process_group(read_insightface_pgid(), logger, process)
    if stopped:
        try:
            INSIGHTFACE_PGID_FILE.unlink()
        except FileNotFoundError:
            pass
    return stopped


class LaunchInsightFaceState(State):
    """Special YASMIN state: Start the appearance pipeline in its own process group."""

    # Special function: Store the ROS node used for logging and launch supervision.
    def __init__(self, node):
        """Initialize the launch state with success and failure outcomes."""
        # Section: The state writes the active process identifier to the Blackboard.
        super().__init__([SUCCEED, FAILED, CANCEL])
        self._node = node
        self.set_description('Starts the InsightFace appearance pipeline as a detached process group.')
        self.add_output_key('active_insightface_pgid', 'Process group for health checks and cleanup.')

    # Special function: Launch the configured appearance pipeline and verify it remains alive.
    def execute(self, blackboard):
        """Start InsightFace and return success only after the startup health check."""
        # Section: Remove any process tree left by an interrupted earlier run.
        stop_registered_insightface(self._node.get_logger())
        command = [
            'ros2',
            'launch',
            'face_detection',
            'appearance_pipeline.launch.py',
            f"personal_test_mode:={str(blackboard['personal_test_mode']).lower()}",
            f"camera_index:={blackboard['camera_index']}",
            f"camera_backend:={blackboard['camera_backend']}",
            f"image_topic:={blackboard['image_topic']}",
            f"enable_depth:={str(blackboard['enable_depth']).lower()}",
            f"face_topic:={blackboard['face_topic']}",
            f"appearance_topic:={blackboard['appearance_topic']}",
            f"show_appearance_gui:={str(blackboard['show_appearance_gui']).lower()}",
            'show_stream_info_overlay:=true',
            'log_video_stats:=true',
            f"stream_info_period_sec:={blackboard['stream_info_period_sec']:.3f}",
            'enable_profile_capture:=false',
            'enable_identification:=false',
        ]
        try:
            process = subprocess.Popen(command, start_new_session=True)
        except OSError as exc:
            self._node.get_logger().error(f'Failed to launch InsightFace: {exc}')
            return FAILED

        # Section: Popen PID is also the new session's process-group identifier.
        blackboard['active_insightface_pgid'] = process.pid
        blackboard['insightface_process'] = process
        write_insightface_pgid(process.pid)
        self._node.get_logger().info(
            f'Started InsightFace process group {process.pid}: {subprocess.list2cmdline(command)}'
        )
        time.sleep(0.7)
        if self.is_canceled():
            return CANCEL
        if process.poll() is not None:
            self._node.get_logger().error('InsightFace exited immediately after launch.')
            return FAILED
        return SUCCEED


class WaitForFaceState(State):
    """Special YASMIN state: Wait until face perception reports detected=true."""

    # Special function: Create the state with the node used for its temporary subscription.
    def __init__(self, node):
        """Initialize face detection waiting outcomes and metadata."""
        # Section: Failure includes timeout, malformed-only input, and detector exit.
        super().__init__([SUCCEED, FAILED, CANCEL])
        self._node = node
        self.set_description('Waits for a detected face JSON message before appearance collection.')

    # Special function: Subscribe for the duration of this state and wait with health checks.
    def execute(self, blackboard):
        """Return success on detected=true and failure on timeout or detector exit."""
        # Section: A threading event safely bridges the executor callback and state thread.
        face_detected = threading.Event()

        # Special function: Parse one face JSON observation without failing on malformed input.
        def face_callback(message):
            """Set the completion event when the message contains detected=true."""
            # Section: Invalid JSON is logged and ignored so later valid messages still work.
            try:
                if bool(json.loads(message.data).get('detected', False)):
                    face_detected.set()
            except (AttributeError, TypeError, json.JSONDecodeError) as exc:
                self._node.get_logger().warning(f'Ignoring malformed face JSON: {exc}')

        topic = blackboard['face_topic']
        timeout = float(blackboard['face_wait_timeout_sec'])
        subscription = self._node.create_subscription(String, topic, face_callback, 10)
        self._node.get_logger().info(f'Waiting up to {timeout:.2f}s for a face on {topic}.')
        start = time.monotonic()
        try:
            while rclpy.ok() and not face_detected.wait(0.1):
                if self.is_canceled():
                    return CANCEL
                if not process_group_running(
                    int(blackboard['active_insightface_pgid']),
                    blackboard_get(blackboard, 'insightface_process', None),
                ):
                    self._node.get_logger().error('InsightFace exited before face detection.')
                    return FAILED
                if timeout > 0.0 and time.monotonic() - start >= timeout:
                    self._node.get_logger().error(f'Timed out waiting for a face on {topic}.')
                    return FAILED
        finally:
            self._node.destroy_subscription(subscription)
        return SUCCEED if face_detected.is_set() else FAILED


class CollectAppearanceState(State):
    """Special YASMIN state: Collect and vote across GuestAppearance messages."""

    # Special function: Retain the ROS node used for subscription and logging.
    def __init__(self, node):
        """Initialize collection outcomes and Blackboard metadata."""
        # Section: This state writes every aggregate used by later reporting states.
        super().__init__([SUCCEED, FAILED, CANCEL])
        self._node = node
        self.set_description('Collects appearance samples for a stable feature vote.')
        for key in (
            'summary_sample_count',
            'shirt_color',
            'shirt_color_confidence',
            'glasses_state',
            'glasses_confidence',
            'hat_state',
            'hat_confidence',
            'face_mask_state',
            'face_mask_confidence',
        ):
            self.add_output_key(key, 'Stable appearance aggregation output.')

    # Special function: Gather samples until both stability and duration requirements are met.
    def execute(self, blackboard):
        """Aggregate available samples, tolerating a partial set at the overall timeout."""
        # Section: A lock protects samples and first-sample time shared with the executor.
        samples = []
        first_sample_time = [None]
        samples_lock = threading.Lock()

        # Special function: Add one immutable message snapshot to the collection window.
        def appearance_callback(message):
            """Record a sample and mark the beginning of the collection window."""
            # Section: ROS messages remain valid after the callback, so retaining them is safe.
            with samples_lock:
                if not samples:
                    first_sample_time[0] = time.monotonic()
                samples.append(message)

        topic = blackboard['appearance_topic']
        timeout = float(blackboard['appearance_wait_timeout_sec'])
        duration = max(0.0, float(blackboard['collection_duration_sec']))
        minimum = max(1, int(blackboard['min_stable_samples']))
        subscription = self._node.create_subscription(
            GuestAppearance, topic, appearance_callback, 10
        )
        start = time.monotonic()
        try:
            while rclpy.ok():
                if self.is_canceled():
                    return CANCEL
                if not process_group_running(
                    int(blackboard['active_insightface_pgid']),
                    blackboard_get(blackboard, 'insightface_process', None),
                ):
                    self._node.get_logger().error('InsightFace exited during appearance collection.')
                    return FAILED
                with samples_lock:
                    sample_count = len(samples)
                    window_elapsed = (
                        first_sample_time[0] is not None
                        and time.monotonic() - first_sample_time[0] >= duration
                    )
                if sample_count >= minimum and window_elapsed:
                    break
                if timeout > 0.0 and time.monotonic() - start >= timeout:
                    if sample_count == 0:
                        self._node.get_logger().error(
                            f'Timed out waiting for appearance samples on {topic}.'
                        )
                        return FAILED
                    self._node.get_logger().warning(
                        f'Collection timed out with {sample_count} sample(s); using them.'
                    )
                    break
                time.sleep(0.1)
        finally:
            self._node.destroy_subscription(subscription)

        # Section: Copy the shared list before aggregation and publish all results atomically by state.
        with samples_lock:
            snapshot = list(samples)
        if not snapshot:
            return FAILED
        aggregate = aggregate_appearance(snapshot)
        for key, value in aggregate.items():
            blackboard[key] = value
        self._node.get_logger().info(
            'Stored appearance summary from '
            f"{aggregate['summary_sample_count']} samples: shirt={aggregate['shirt_color'] or 'unknown'} "
            f"glasses={aggregate['glasses_state']} hat={aggregate['hat_state']} "
            f"face_mask={aggregate['face_mask_state']}."
        )
        return SUCCEED


class BuildSummaryState(State):
    """Special YASMIN state: Convert Blackboard features into one speech string."""

    # Special function: Retain the ROS node used to report generated speech.
    def __init__(self, node):
        """Initialize the deterministic summary-building state."""
        # Section: Summary construction has no external interface and normally cannot fail.
        super().__init__([SUCCEED, FAILED])
        self._node = node
        self.set_description('Builds natural-language text from stable appearance features.')
        self.add_output_key('feature_summary_text', 'Text sent once to the speech service.')

    # Special function: Read aggregate fields and write the generated text to the Blackboard.
    def execute(self, blackboard):
        """Generate the summary while converting missing data to UNKNOWN semantics."""
        # Section: Defensive defaults make the cleanup transition reachable on partial data.
        try:
            text = build_feature_summary_text(
                blackboard_get(blackboard, 'shirt_color', ''),
                blackboard_get(blackboard, 'glasses_state', GuestAppearance.UNKNOWN),
                blackboard_get(blackboard, 'hat_state', GuestAppearance.UNKNOWN),
                blackboard_get(blackboard, 'face_mask_state', GuestAppearance.UNKNOWN),
            )
        except (RuntimeError, TypeError, ValueError) as exc:
            self._node.get_logger().error(f'Failed to build feature summary: {exc}')
            return FAILED
        blackboard['feature_summary_text'] = text
        self._node.get_logger().info(f'Generated feature summary: {text}')
        return SUCCEED


class SpeakSummaryState(State):
    """Special YASMIN state: Call SayText exactly once with the generated summary."""

    # Special function: Store the ROS node used for the asynchronous service client.
    def __init__(self, node):
        """Initialize speech outcomes and input metadata."""
        # Section: The executor thread completes the future while this state waits.
        super().__init__([SUCCEED, FAILED, CANCEL])
        self._node = node
        self.set_description('Sends the generated summary to the configured TTS service once.')
        self.add_input_key('feature_summary_text', 'Generated appearance summary text.')

    # Special function: Wait for TTS, send one request, and validate its response.
    def execute(self, blackboard):
        """Return success only when SayText reports successful speech completion."""
        # Section: Refuse an empty request before creating the ROS service client.
        text = str(blackboard_get(blackboard, 'feature_summary_text', ''))
        if not text:
            self._node.get_logger().error('No feature summary text is available for TTS.')
            return FAILED
        service_name = blackboard['tts_service_name']
        client = self._node.create_client(SayText, service_name)
        try:
            if not client.wait_for_service(timeout_sec=10.0):
                self._node.get_logger().error(f'{service_name} is unavailable.')
                return FAILED
            request = SayText.Request()
            request.text = text
            future = client.call_async(request)
            deadline = time.monotonic() + 30.0
            while rclpy.ok() and not future.done():
                if self.is_canceled():
                    return CANCEL
                if time.monotonic() >= deadline:
                    self._node.get_logger().error(f'{service_name} call timed out.')
                    return FAILED
                time.sleep(0.1)
            if not future.done():
                return FAILED
            response = future.result()
            if response is None or not response.success:
                message = response.message if response is not None else 'no response'
                self._node.get_logger().error(f'{service_name} failed: {message}')
                return FAILED
            self._node.get_logger().info(f'TTS completed: {response.message}')
            return SUCCEED
        finally:
            self._node.destroy_client(client)


class CleanupState(State):
    """Special YASMIN state: Always stop InsightFace before a terminal outcome."""

    # Special function: Configure cleanup for either the success or abort branch.
    def __init__(self, node, terminal_outcome):
        """Retain the terminal outcome that follows successful cleanup."""
        # Section: Cleanup returns its branch outcome so transitions remain explicit in the viewer.
        super().__init__([terminal_outcome])
        self._node = node
        self._terminal_outcome = terminal_outcome
        self.set_description('Stops the InsightFace process tree before the machine exits.')

    # Special function: Stop the detector process group and clear Blackboard process state.
    def execute(self, blackboard):
        """Perform idempotent process cleanup and return the configured terminal branch."""
        # Section: Both success and error paths use exactly the same safety cleanup.
        process = blackboard_get(blackboard, 'insightface_process', None)
        stop_registered_insightface(self._node.get_logger(), process)
        blackboard['active_insightface_pgid'] = -1
        if process is not None:
            try:
                process.wait(timeout=0.1)
            except subprocess.TimeoutExpired:
                pass
        return self._terminal_outcome


# Special function: Declare ROS parameters and copy the run configuration into a Blackboard.
def create_blackboard(node):
    """Build the shared YASMIN data store from the existing launch parameter contract."""
    # Section: Parameter names and defaults remain compatible with the SMACC2 implementation.
    defaults = {
        'personal_test_mode': True,
        'camera_index': '0',
        'camera_backend': 'realsense',
        'image_topic': '/camera/camera/color/image_raw',
        'enable_depth': True,
        'face_topic': '/perception/largest_face',
        'appearance_topic': '/appearance/current_estimate',
        'tts_service_name': '/voice/tts/say',
        'face_wait_timeout_sec': 10.0,
        'appearance_wait_timeout_sec': 10.0,
        'collection_duration_sec': 2.0,
        'min_stable_samples': 3,
        'show_appearance_gui': True,
        'stream_info_period_sec': 1.0,
    }
    blackboard = Blackboard()
    for name, default in defaults.items():
        node.declare_parameter(name, default)
        blackboard[name] = node.get_parameter(name).value

    # Section: Result and process keys are initialized for viewer metadata and cleanup safety.
    blackboard['active_insightface_pgid'] = -1
    blackboard['summary_sample_count'] = 0
    blackboard['shirt_color'] = ''
    blackboard['shirt_color_confidence'] = 0.0
    blackboard['glasses_state'] = int(GuestAppearance.UNKNOWN)
    blackboard['glasses_confidence'] = 0.0
    blackboard['hat_state'] = int(GuestAppearance.UNKNOWN)
    blackboard['hat_confidence'] = 0.0
    blackboard['face_mask_state'] = int(GuestAppearance.UNKNOWN)
    blackboard['face_mask_confidence'] = 0.0
    blackboard['feature_summary_text'] = ''
    return blackboard


# Special function: Construct the explicit YASMIN state graph and cleanup transitions.
def create_state_machine(node):
    """Return the feature-summary machine with succeeded and aborted terminal outcomes."""
    # Section: Every operational failure converges on cleanup before reaching ABORT.
    machine = StateMachine(outcomes=[SUCCEED, ABORT], handle_sigint=True)
    machine.set_name('INSIGHTFACE_FEATURE_SUMMARY')
    machine.set_description(
        'Launches InsightFace, collects stable appearance features, and speaks one summary.'
    )
    machine.add_state(
        'LAUNCH_INSIGHTFACE',
        LaunchInsightFaceState(node),
        transitions={SUCCEED: 'WAIT_FOR_FACE', FAILED: 'CLEANUP_ABORT', CANCEL: 'CLEANUP_ABORT'},
    )
    machine.add_state(
        'WAIT_FOR_FACE',
        WaitForFaceState(node),
        transitions={SUCCEED: 'COLLECT_APPEARANCE', FAILED: 'CLEANUP_ABORT', CANCEL: 'CLEANUP_ABORT'},
    )
    machine.add_state(
        'COLLECT_APPEARANCE',
        CollectAppearanceState(node),
        transitions={SUCCEED: 'BUILD_SUMMARY', FAILED: 'CLEANUP_ABORT', CANCEL: 'CLEANUP_ABORT'},
    )
    machine.add_state(
        'BUILD_SUMMARY',
        BuildSummaryState(node),
        transitions={SUCCEED: 'SPEAK_SUMMARY', FAILED: 'CLEANUP_ABORT'},
    )
    machine.add_state(
        'SPEAK_SUMMARY',
        SpeakSummaryState(node),
        transitions={SUCCEED: 'CLEANUP_SUCCESS', FAILED: 'CLEANUP_ABORT', CANCEL: 'CLEANUP_ABORT'},
    )
    machine.add_state(
        'CLEANUP_SUCCESS',
        CleanupState(node, SUCCEED),
        transitions={SUCCEED: SUCCEED},
    )
    machine.add_state(
        'CLEANUP_ABORT',
        CleanupState(node, ABORT),
        transitions={ABORT: ABORT},
    )
    return machine


# Special function: Run the ROS executor, YASMIN viewer, and state machine with guaranteed cleanup.
def main(args=None):
    """Execute one lifecycle-supervised feature-summary run."""
    # Section: ROS logging is routed through YASMIN while callbacks run on a background executor.
    rclpy.init(args=args)
    set_ros_loggers()
    node = Node('feature_summary_yasmin_node')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    blackboard = create_blackboard(node)
    machine = create_state_machine(node)
    viewer = YasminViewerPub(machine, 'INSIGHTFACE_FEATURE_SUMMARY', node=node)
    outcome = ABORT
    try:
        outcome = machine(blackboard)
        node.get_logger().info(f'Feature summary YASMIN outcome: {outcome}')
    except Exception as exc:  # pylint: disable=broad-except
        node.get_logger().error(f'Feature summary YASMIN failed: {exc}')
    finally:
        # Section: Process cleanup is repeated defensively for exceptions outside normal transitions.
        stop_registered_insightface(
            node.get_logger(), blackboard_get(blackboard, 'insightface_process', None)
        )
        viewer.shutdown()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if outcome == SUCCEED else 1


# Section: ROS 2 invokes the state machine through the installed console-style script.
if __name__ == '__main__':
    raise SystemExit(main())
