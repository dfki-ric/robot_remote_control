/**
 * Web interface for robots using robot_remote_control.
 *
 * The page has to be served by the TransportHttp of the robot (commands are sent to the same server),
 * the telemetry websocket port is given as url parameter: index.html?telemetryport=7002
 */
import { RobotRemoteControl, LatestValueSender, LOG_LEVELS, TELEMETRY_TYPES, base64ToBytes } from './rrc-client.js';
import { Viewer } from './viewer.js';
import { loadRobotModel } from './model-loader.js';
import { Joystick, KeyboardInput, readGamepad } from './input.js';

const $ = (id) => document.getElementById(id);

const params = new URLSearchParams(window.location.search);
const host = window.location.hostname || '127.0.0.1';
const commandPort = params.get('commandport') || window.location.port || '80';
const telemetryPort = params.get('telemetryport') || '7002';

const rrc = new RobotRemoteControl(host, commandPort, telemetryPort);
const viewer = new Viewer($('viewer'));

const state = {
    robotName: null,
    worldFrame: '',          // frame of the current pose, used for goals
    channels: [],            // ChannelDefinitions
    controllableFrames: [],
    controllableJoints: null,
    jointState: null,
    dynamicSimpleActions: false,
    modelFiles: null,
};

// ------------------------------------------------------------------ helpers

function element(tag, attributes = {}, ...children) {
    const node = document.createElement(tag);
    for (const [key, value] of Object.entries(attributes)) {
        if (value === undefined || value === null) {
            continue;
        }
        if (key === 'class') {
            node.className = value;
        } else if (key.startsWith('on')) {
            node.addEventListener(key.slice(2), value);
        } else {
            node.setAttribute(key, value);
        }
    }
    children.flat().forEach((child) => {
        if (child !== undefined && child !== null && child !== '') {
            node.append(child instanceof Node ? child : document.createTextNode(String(child)));
        }
    });
    return node;
}

function toast(message) {
    const node = element('div', {class: 'toast'}, message);
    $('toasts').append(node);
    setTimeout(() => node.remove(), 6000);
}

/**
 * send a command, errors are shown to the user
 */
async function send(type, payload) {
    try {
        return await rrc.command(type, payload);
    } catch (error) {
        toast(error.name === 'AbortError' ? `${type}: no reply from the robot` : error.message);
        throw error;
    }
}

function request(type, channel = 0, timeout = 5000) {
    return rrc.requestTelemetry(type, channel, timeout).catch(() => null);
}

function format(value, digits = 2) {
    return Number.isFinite(value) ? value.toFixed(digits) : '–';
}

function xyz(v, digits = 2) {
    v = v || {};
    return `${format(v.x || 0, digits)} ${format(v.y || 0, digits)} ${format(v.z || 0, digits)}`;
}

const degrees = (radians) => radians * 180 / Math.PI;

function yawOf(q) {
    q = q || {};
    const [x, y, z, w] = [q.x || 0, q.y || 0, q.z || 0, q.w || 0];
    if (x * x + y * y + z * z + w * w < 1e-9) {
        return 0;
    }
    return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

function channelName(type, channel) {
    const typeId = TELEMETRY_TYPES.indexOf(type);
    const definition = state.channels.find((c) => (c.messagetype || 0) === typeId && (c.channelno || 0) === channel);
    return definition ? definition.name : '';
}

function timeOf(header) {
    const stamp = header && header.timestamp;
    const date = stamp && stamp.secs ? new Date(stamp.secs * 1000 + (stamp.nsecs || 0) / 1e6) : new Date();
    return date.toLocaleTimeString();
}

/**
 * calls render at most every interval ms with the latest arguments
 */
function throttle(render, interval) {
    let last = 0;
    let timer = null;
    let latest = null;
    return (...args) => {
        latest = args;
        const wait = last + interval - performance.now();
        if (wait <= 0) {
            last = performance.now();
            render(...latest);
        } else if (!timer) {
            timer = setTimeout(() => {
                timer = null;
                last = performance.now();
                render(...latest);
            }, wait);
        }
    };
}

// ------------------------------------------------------------------ connection

rrc.onStatus((status) => {
    const dot = (id, ok, error) => {
        $(id).className = 'dot' + (ok ? ' ok' : error ? ' error' : '');
    };
    dot('status-commands', status.commands === 'online', status.commands === 'offline');
    dot('status-telemetry', status.telemetry === 'connected', status.telemetry === 'disconnected');
    $('latency').textContent = status.latency !== null ? `${status.latency} ms` : '';
});

let telemetryWasConnected = false;
rrc.onStatus((status) => {
    if (status.telemetry === 'connected') {
        // reload after a reconnect, the robot may have been restarted
        if (telemetryWasConnected === null) {
            loadInterface();
        }
        telemetryWasConnected = true;
    } else if (telemetryWasConnected) {
        telemetryWasConnected = null;
    }
});

let loadingInterface = false;

async function loadInterface() {
    if (loadingInterface) {
        return;
    }
    loadingInterface = true;
    try {
        const [name, options, channels, frames, joints, actions, complex, streams, files, robotState] = await Promise.all([
            request('ROBOT_NAME'), request('INTERFACE_OPTIONS'), request('CHANNELS_DEFINITION'),
            request('CONTROLLABLE_FRAMES'), request('CONTROLLABLE_JOINTS'), request('SIMPLE_ACTIONS'),
            request('COMPLEX_ACTIONS'), request('VIDEO_STREAMS'), request('FILE_DEFINITION'), request('ROBOT_STATE'),
        ]);
        if (rrc.status.commands !== 'online') {
            $('robot-name').textContent = 'robot not reachable';
            $('viewer-message').textContent = `no connection to ${rrc.commandUrl}`;
            return;
        }
        const robotName = (name && name.value) || '';
        const otherRobot = state.robotName !== null && state.robotName !== robotName;
        state.robotName = robotName;
        $('robot-name').textContent = robotName || 'unnamed robot';
        document.title = `${robotName || 'Robot'} · Robot Remote Control`;
        state.dynamicSimpleActions = Boolean(options && options.dynamicSimpleActions);
        state.channels = (channels && channels.channel) || [];

        setControllableFrames(frames);
        setControllableJoints(joints);
        setSimpleActions(actions);
        setComplexActions(complex);
        setVideoStreams(streams);
        setFiles(files);
        setMaps();
        if (robotState) {
            setRobotState(robotState);
        }
        setInfo('name', robotName || '–');
        setInfo('commands', rrc.commandUrl);
        setInfo('telemetry', rrc.telemetryUrl);
        setInfo('options', options ? Object.keys(options).filter((key) => options[key]).join(', ') || 'none' : '–');
        loadVersions();
        if (!state.modelFiles || otherRobot) {
            loadModel();
        }
    } finally {
        loadingInterface = false;
    }
}

async function loadVersions() {
    for (const [key, type] of [['library', 'LIBRARY_VERSION'], ['protocol', 'PROTOCOL_VERSION'], ['git', 'GIT_VERSION']]) {
        try {
            const version = await rrc.command(type);
            setInfo(key, type === 'LIBRARY_VERSION' ? version : version.slice(0, 12));
        } catch (error) {
            setInfo(key, '–');
        }
    }
}

async function loadModel() {
    const message = $('viewer-message');
    try {
        const model = await loadRobotModel(rrc, (text) => {
            message.textContent = `${text}…`;
        });
        message.textContent = '';
        if (!model) {
            setInfo('model', 'none');
            return;
        }
        if (state.modelFiles) {
            state.modelFiles.dispose();
        }
        state.modelFiles = model.files;
        viewer.setRobotModel(model.robot);
        if (state.jointState) {
            viewer.setJointState(state.jointState);
        }
        // the joint limits are known now
        setControllableJoints(state.controllableJoints);
        const movable = Object.values(model.robot.joints).filter((joint) => joint.jointType !== 'fixed').length;
        setInfo('model', `${model.robot.robotName || 'unnamed'}, ${movable} movable joints`);
        if (model.missing.length) {
            toast(`${model.missing.length} model file(s) could not be loaded: ${model.missing.slice(0, 3).join(', ')}`);
        }
    } catch (error) {
        message.textContent = `model: ${error.message}`;
        setInfo('model', 'failed');
    }
}

// ------------------------------------------------------------------ robot state, pose

function setRobotState(robotState) {
    const names = robotState.stateName || [];
    const chips = (robotState.state || []).map((value, i) =>
        element('span', {class: 'chip', title: names[i] || ''}, names[i] ? element('span', {class: 'label'}, names[i]) : '', value));
    $('robot-state').replaceChildren(...chips);
}

rrc.on('ROBOT_STATE', setRobotState);

rrc.on('CURRENT_POSE', (pose) => {
    state.worldFrame = (pose.header && pose.header.frame) || '';
    viewer.setPose(pose);
    showPose(pose);
});

const showPose = throttle((pose) => {
    const p = pose.position || {};
    $('pose-display').textContent =
        `x ${format(p.x || 0)}  y ${format(p.y || 0)}  z ${format(p.z || 0)}  yaw ${format(degrees(yawOf(pose.orientation)), 1)}°`;
}, 100);

rrc.on('POSES', (poses) => viewer.setPath(poses));
rrc.on('LASER_SCAN', (scan) => viewer.setLaserScan(scan));
rrc.on('POINTCLOUD', (cloud) => viewer.setPointCloud(cloud));
rrc.on('MAP', (map, channel) => {
    // maps are usually requested, but may also be sent as telemetry
    if (map.map) {
        try {
            viewer.setMap(`map:${channel}`, map.map);
        } catch (error) {
            console.warn(error);
        }
    }
});

// ------------------------------------------------------------------ viewer toolbar, goals

let pickMode = null;

function setPickMode(mode) {
    pickMode = mode;
    $('goal-mode').classList.toggle('active', mode === 'goal');
    $('task-pick').classList.toggle('active', mode === 'waypoints');
    $('pick-hint').textContent = {
        goal: 'click on the ground to set a goal, drag to set its orientation · Esc to cancel',
        waypoints: 'click on the ground to add points · Esc to finish',
    }[mode] || '';
    viewer.setPickMode(mode, mode === 'goal' ? sendGoal : mode === 'waypoints' ? addTaskPoint : null);
}

function poseMessage(x, y, yaw) {
    const pose = {header: {frame: state.worldFrame}, position: {x, y, z: 0}};
    if (yaw !== null && yaw !== undefined) {
        pose.orientation = {x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2)};
    }
    return pose;
}

function supportsControl(type) {
    return state.controllableFrames.some((frame) => (frame.type || []).includes(type));
}

async function sendGoal(x, y, yaw) {
    setPickMode(null);
    const pose = poseMessage(x, y, yaw);
    // robots without information on controllable frames get a target pose
    if (state.controllableFrames.length && !supportsControl('POSE_CONTROL') && supportsControl('GOTO_CONTROL')) {
        await send('GOTO_COMMAND', {waypointPose: pose, maxForwardSpeed: Number($('linear-speed').value)}).catch(() => {});
    } else {
        await send('TARGET_POSE_COMMAND', pose).catch(() => {});
    }
}

$('goal-mode').addEventListener('click', () => setPickMode(pickMode === 'goal' ? null : 'goal'));
$('follow').addEventListener('click', () => {
    viewer.follow = !viewer.follow;
    $('follow').classList.toggle('active', viewer.follow);
});
$('reset-view').addEventListener('click', () => viewer.resetView());
$('clear-trail').addEventListener('click', () => viewer.clearTrail());
document.querySelectorAll('[data-layer]').forEach((checkbox) => {
    checkbox.addEventListener('change', () => viewer.setLayerVisible(checkbox.dataset.layer, checkbox.checked));
});

// ------------------------------------------------------------------ drive

// up/down: forward/backward, left/right: turn
const moveJoystick = new Joystick($('move-joystick'), 'xy');
// left/right: sideways (holonomic robots)
const lateralJoystick = new Joystick($('lateral-joystick'), 'x');
const keyboard = new KeyboardInput();
// one twist request at a time, in order
const twistSender = new LatestValueSender((twist) => rrc.command('TWIST_COMMAND', twist));
let driving = false;

function bindSlider(id, digits, unit) {
    const input = $(id);
    const label = $(`${id}-value`);
    const update = () => {
        label.textContent = `${Number(input.value).toFixed(digits)} ${unit}`;
    };
    input.addEventListener('input', update);
    update();
    return input;
}

const linearSpeed = bindSlider('linear-speed', 2, 'm/s');
const angularSpeed = bindSlider('angular-speed', 2, 'rad/s');

function twistMessage(forward, left, turn) {
    const linear = Number(linearSpeed.value);
    const angular = Number(angularSpeed.value);
    return {
        header: {frame: $('twist-frame').value},
        linear: {x: forward * linear, y: left * linear, z: 0},
        angular: {x: 0, y: 0, z: turn * angular},
    };
}

function stopDriving() {
    driving = false;
    twistSender.set(twistMessage(0, 0, 0));
}

function driveInput() {
    if (moveJoystick.active || lateralJoystick.active) {
        return {forward: moveJoystick.value.y, left: -lateralJoystick.value.x, turn: -moveJoystick.value.x};
    }
    const keys = keyboard.value;
    if (keys.forward || keys.left || keys.turn) {
        return keys;
    }
    return readGamepad() || {forward: 0, left: 0, turn: 0};
}

// while there is input, the twist is repeated (robots may stop without new commands)
setInterval(() => {
    const input = driveInput();
    if (Math.abs(input.forward) + Math.abs(input.left) + Math.abs(input.turn) > 0.01) {
        driving = true;
        twistSender.set(twistMessage(input.forward, input.left, input.turn));
    } else if (driving) {
        stopDriving();
    }
}, 100);

$('stop').addEventListener('click', stopDriving);
window.addEventListener('keydown', (event) => {
    if (keyboard.isTyping(event)) {
        return;
    }
    if (event.code === 'Space') {
        event.preventDefault();
        stopDriving();
    } else if (event.code === 'Escape') {
        setPickMode(null);
    }
});

function setControllableFrames(frames) {
    state.controllableFrames = (frames && frames.controllableFrames) || [];
    const select = $('twist-frame');
    const selected = select.value;
    select.replaceChildren(element('option', {value: ''}, 'default'));
    state.controllableFrames
        .filter((frame) => (frame.type || []).includes('TWIST_CONTROL'))
        .forEach((frame) => select.append(element('option', {value: frame.frame}, frame.frame)));
    select.value = Array.from(select.options).some((option) => option.value === selected) ? selected : '';
    setInfo('frames', state.controllableFrames.map((frame) => `${frame.frame} (${(frame.type || []).map((t) => t.replace('_CONTROL', '').toLowerCase()).join(', ')})`).join('; ') || 'none');
}

rrc.on('CURRENT_TWIST', throttle((twist) => {
    const linear = twist.linear || {};
    const angular = twist.angular || {};
    $('velocity-display').textContent =
        `velocity  x ${format(linear.x || 0)}  y ${format(linear.y || 0)} m/s  turn ${format(angular.z || 0)} rad/s`;
}, 200));

// ------------------------------------------------------------------ simple actions

const actionControls = new Map();
let actionSignature = '';

function setSimpleActions(actions) {
    if (!actions) {
        return;
    }
    const list = actions.actions || [];
    const signature = JSON.stringify(list.map((action) => [action.name, action.type]));
    if (signature !== actionSignature) {
        actionSignature = signature;
        buildSimpleActions(list);
    }
    const states = {};
    list.forEach((action) => {
        states[action.name] = action.state || 0;
    });
    list.forEach((action) => {
        const control = actionControls.get(action.name);
        control.update(action.state || 0);
        // shown only while the actions they depend on are in the required state
        const visible = control.dependencies.every((dependency) =>
            states[dependency.dependsOnAction] === (dependency.dependsOnActionInState || 0));
        control.node.classList.toggle('hidden', !visible);
    });
}

function buildSimpleActions(list) {
    const container = $('simple-actions');
    actionControls.clear();
    if (!list.length) {
        container.replaceChildren(element('span', {class: 'muted'}, 'no simple actions'));
        return;
    }
    const triggers = element('div', {class: 'row'});
    const rows = element('div');
    list.forEach((action) => {
        const type = action.type || {};
        const documentation = type.documentation || '';
        const dependencies = type.actionDependency || [];
        if (type.type === 'TRIGGER') {
            const button = element('button', {title: documentation, onclick: () => sendSimpleAction(action.name, 1)}, action.name);
            triggers.append(button);
            actionControls.set(action.name, {node: button, dependencies, update: () => {}});
            return;
        }
        let control;
        let update;
        if (type.valueNames && type.valueNames.length) {
            const buttons = type.valueNames.map((named) =>
                element('button', {onclick: () => sendSimpleAction(action.name, named.value || 0)}, named.name));
            control = element('div', {class: 'segmented'}, buttons);
            update = (value) => buttons.forEach((button, i) => button.classList.toggle('active', (type.valueNames[i].value || 0) === value));
        } else {
            const min = type.minState || 0;
            const max = type.maxState || 0;
            const step = type.stepSize || (type.type === 'VALUE_INT' ? 1 : 'any');
            const number = element('input', {type: 'number', min, max: max > min ? max : null, step});
            const range = max > min ? element('input', {type: 'range', min, max, step}) : null;
            const commit = (value) => sendSimpleAction(action.name, Number(value));
            number.addEventListener('change', () => {
                if (range) {
                    range.value = number.value;
                }
                commit(number.value);
            });
            if (range) {
                range.addEventListener('input', () => {
                    number.value = range.value;
                });
                range.addEventListener('change', () => commit(range.value));
            }
            control = element('div', {class: 'slider-row'}, range, number);
            update = (value) => {
                // do not overwrite user input
                if (document.activeElement !== number && document.activeElement !== range) {
                    number.value = value;
                    if (range) {
                        range.value = value;
                    }
                }
            };
        }
        const row = element('div', {class: 'action'}, element('label', {title: documentation || action.name}, action.name), control);
        rows.append(row);
        actionControls.set(action.name, {node: row, dependencies, update});
    });
    container.replaceChildren(...(triggers.childElementCount ? [triggers] : []), rows);
}

async function sendSimpleAction(name, value) {
    try {
        await send('SIMPLE_ACTIONS_COMMAND', {name, state: value});
    } catch (error) {
        return;
    }
    if (!state.dynamicSimpleActions) {
        // the robot does not send updates, the state may still change
        setSimpleActions(await request('SIMPLE_ACTIONS'));
    }
}

rrc.on('SIMPLE_ACTIONS', setSimpleActions);

// ------------------------------------------------------------------ joints

const jointControls = new Map();
const jointSender = new LatestValueSender((command) => rrc.command('JOINTS_COMMAND', command));

function setControllableJoints(joints) {
    state.controllableJoints = joints;
    const names = (joints && joints.name) || [];
    const container = $('joints');
    jointControls.clear();
    if (!names.length) {
        container.replaceChildren(element('span', {class: 'muted'}, 'no controllable joints'));
        return;
    }
    container.replaceChildren(...names.map((name) => {
        const limits = viewer.jointLimits(name) || {type: 'revolute', lower: -Math.PI, upper: Math.PI};
        const lower = limits.upper > limits.lower ? limits.lower : -Math.PI;
        const upper = limits.upper > limits.lower ? limits.upper : Math.PI;
        const range = element('input', {type: 'range', min: lower, max: upper, step: (upper - lower) / 1000, value: 0});
        const value = element('span', {}, '–');
        const control = {range, value, prismatic: limits.type === 'prismatic', active: false};
        range.addEventListener('pointerdown', () => {
            control.active = true;
        });
        range.addEventListener('pointerup', () => {
            control.active = false;
        });
        range.addEventListener('input', () => {
            jointSender.set({name: [name], position: [Number(range.value)]});
        });
        jointControls.set(name, control);
        return element('div', {class: 'joint'}, element('div', {class: 'name'}, element('span', {}, name), value), range);
    }));
    if (state.jointState) {
        updateJoints(state.jointState);
    }
}

function updateJoints(jointState) {
    const names = jointState.name || [];
    const positions = jointState.position || [];
    names.forEach((name, i) => {
        const control = jointControls.get(name);
        if (!control || i >= positions.length) {
            return;
        }
        const position = positions[i];
        control.value.textContent = control.prismatic
            ? `${format(position, 3)} m`
            : `${format(position, 2)} rad  ${format(degrees(position), 0)}°`;
        if (!control.active && document.activeElement !== control.range) {
            control.range.value = position;
        }
    });
}

const updateJointsThrottled = throttle(updateJoints, 100);

rrc.on('JOINT_STATE', (jointState) => {
    state.jointState = jointState;
    viewer.setJointState(jointState);
    updateJointsThrottled(jointState);
});

// ------------------------------------------------------------------ camera

const imageSources = new Map();

function addImageSource(key, label, image) {
    const select = $('image-source');
    if (!imageSources.has(key)) {
        if (!imageSources.size) {
            select.replaceChildren();
        }
        select.append(element('option', {value: key}, label));
    }
    imageSources.set(key, image);
    if (select.value === key) {
        drawSelectedImage();
    }
}

rrc.on('IMAGE', (image, channel) => {
    addImageSource(`image:${channel}`, channelName('IMAGE', channel) || `image ${channel}`, image);
});

rrc.on('IMAGE_LAYERS', (layers, channel) => {
    (layers.layers || []).forEach((image, i) => {
        addImageSource(`layers:${channel}:${i}`, `${channelName('IMAGE_LAYERS', channel) || `layers ${channel}`}, ${i}: ${image.encoding || ''}`, image);
    });
});

$('image-source').addEventListener('change', () => drawSelectedImage());

const drawSelectedImage = throttle(() => {
    const image = imageSources.get($('image-source').value);
    if (image && $('camera-image').closest('details').open) {
        drawImage(image).catch((error) => {
            $('image-info').textContent = error.message;
        });
    }
}, 100);

const PIXEL_FORMATS = {
    MODE_RGB: [3, [0, 1, 2]], RGB8: [3, [0, 1, 2]],
    MODE_BGR: [3, [2, 1, 0]], BGR8: [3, [2, 1, 0]],
    MODE_RGB32: [4, [0, 1, 2]], RGBA8: [4, [0, 1, 2]], BGRA8: [4, [2, 1, 0]],
    MODE_GRAYSCALE: [1, [0, 0, 0]], MONO8: [1, [0, 0, 0]],
};

async function drawImage(image) {
    const canvas = $('camera-image');
    const context = canvas.getContext('2d');
    const encoding = (image.encoding || '').toUpperCase();
    const bytes = base64ToBytes(image.data);
    const width = image.width || 0;
    const height = image.height || 0;
    $('image-info').textContent = `${width}×${height} ${image.encoding || ''}`;

    if (/JPEG|JPG|PNG/.test(encoding)) {
        const bitmap = await createImageBitmap(new Blob([bytes]));
        canvas.width = bitmap.width;
        canvas.height = bitmap.height;
        context.drawImage(bitmap, 0, 0);
        return;
    }
    if (!width || !height) {
        throw new Error('empty image');
    }
    const pixels = context.createImageData(width, height);
    const out = pixels.data;
    if (PIXEL_FORMATS[encoding]) {
        const [channels, order] = PIXEL_FORMATS[encoding];
        const step = image.step || width * channels;
        for (let y = 0; y < height; ++y) {
            for (let x = 0; x < width; ++x) {
                const source = y * step + x * channels;
                const target = (y * width + x) * 4;
                out[target] = bytes[source + order[0]];
                out[target + 1] = bytes[source + order[1]];
                out[target + 2] = bytes[source + order[2]];
                out[target + 3] = 255;
            }
        }
    } else if (['32FC1', 'MONO16', '16UC1'].includes(encoding)) {
        // depth images, scaled to the value range
        const view = new DataView(bytes.buffer);
        const littleEndian = !image.isBigendian;
        const float = encoding === '32FC1';
        const size = float ? 4 : 2;
        const step = image.step || width * size;
        const values = new Float32Array(width * height);
        let min = Infinity;
        let max = -Infinity;
        for (let y = 0; y < height; ++y) {
            for (let x = 0; x < width; ++x) {
                const offset = y * step + x * size;
                const v = float ? view.getFloat32(offset, littleEndian) : view.getUint16(offset, littleEndian);
                values[y * width + x] = v;
                if (Number.isFinite(v) && v > 0) {
                    min = Math.min(min, v);
                    max = Math.max(max, v);
                }
            }
        }
        const range = max > min ? max - min : 1;
        values.forEach((v, i) => {
            const shade = Number.isFinite(v) && v > 0 ? 255 - 225 * (v - min) / range : 0;
            out.set([shade, shade, shade, 255], i * 4);
        });
        $('image-info').textContent += `  ${format(min)}…${format(max)}`;
    } else {
        throw new Error(`unsupported image encoding ${image.encoding}`);
    }
    canvas.width = width;
    canvas.height = height;
    context.putImageData(pixels, 0, 0);
}

function setVideoStreams(streams) {
    const list = (streams && streams.stream) || [];
    $('video-streams').replaceChildren(...list.map((stream) =>
        element('div', {class: 'small'}, 'video stream: ', element('a', {href: stream.url, target: '_blank'}, stream.url))));
}

// ------------------------------------------------------------------ sensors

const sensorNodes = new Map();

function setSensor(key, title, rows) {
    let node = sensorNodes.get(key);
    if (!node) {
        node = element('div', {class: 'sensor'});
        sensorNodes.set(key, node);
        const container = $('sensors');
        if (sensorNodes.size === 1) {
            container.replaceChildren();
        }
        container.append(node);
    }
    node.replaceChildren(
        element('div', {class: 'title'}, title),
        element('table', {}, rows.map(([name, value]) => element('tr', {}, element('td', {}, name), element('td', {}, value)))));
}

const sensorUpdates = new Map();

function updateSensor(key, ...args) {
    if (!sensorUpdates.has(key)) {
        sensorUpdates.set(key, throttle(setSensor, 250));
    }
    sensorUpdates.get(key)(key, ...args);
}

rrc.on('SIMPLE_SENSOR', (sensor, channel) => {
    const values = sensor.value || [];
    const names = sensor.valueNames || [];
    const title = sensor.name || channelName('SIMPLE_SENSOR', channel) || `sensor ${channel}`;
    let rows;
    if (values.length <= 12) {
        rows = values.map((value, i) => [names[i] || `value ${i}`, format(value, 3)]);
    } else {
        const finite = values.filter(Number.isFinite);
        const min = finite.reduce((a, b) => Math.min(a, b), Infinity);
        const max = finite.reduce((a, b) => Math.max(a, b), -Infinity);
        const size = sensor.size || {};
        rows = [[`grid ${size.x || values.length}×${size.y || 1}`, `${format(min)} … ${format(max)}`]];
    }
    updateSensor(`simple:${channel}`, title, rows);
});

rrc.on('IMU_VALUES', (imu, channel) => {
    updateSensor(`imu:${channel}`, `IMU ${(imu.header && imu.header.frame) || ''}`, [
        ['acceleration (m/s²)', xyz(imu.acceleration)],
        ['gyro (rad/s)', xyz(imu.gyro, 3)],
        ['magnetometer', xyz(imu.mag)],
    ]);
});

rrc.on('WRENCH_STATE', (wrenches, channel) => {
    (wrenches.wrenches || []).forEach((wrench, i) => {
        updateSensor(`wrench:${channel}:${i}`, `wrench ${(wrench.header && wrench.header.frame) || i}`, [
            ['force (N)', xyz(wrench.force)],
            ['torque (Nm)', xyz(wrench.torque, 3)],
        ]);
    });
});

// ------------------------------------------------------------------ tasks (complex actions)

const POSE_TASKS = ['POSE', 'POSE_LIST', 'AREA', 'VOLUME'];
const TWIST_TASKS = ['TWIST', 'TWIST_LIST'];
let complexActions = [];
let taskPoints = [];
let taskTwists = [];

function setComplexActions(actions) {
    complexActions = (actions && actions.actions) || [];
    const select = $('complex-action');
    select.replaceChildren(...complexActions.map((action, i) =>
        element('option', {value: i}, `${action.name} (${(action.type || 'UNDEFINED').toLowerCase()})`)));
    if (!complexActions.length) {
        select.append(element('option', {value: ''}, 'no complex actions'));
    }
    updateTaskForm();
}

function selectedTask() {
    return complexActions[Number($('complex-action').value)];
}

function updateTaskForm() {
    const task = selectedTask();
    const type = task && task.type;
    $('task-poses').classList.toggle('hidden', !POSE_TASKS.includes(type));
    $('task-twists').classList.toggle('hidden', !TWIST_TASKS.includes(type));
    $('task-send').disabled = !task;
    $('task-count').textContent = `${taskPoints.length} point(s)`;
    $('task-twist-list').textContent = taskTwists.map((t) => `(${t.linear.x}, ${t.linear.y}, ${t.angular.z})`).join(' ');
    viewer.setWaypoints(POSE_TASKS.includes(type) ? taskPoints : []);
}

function addTaskPoint(x, y) {
    const task = selectedTask();
    if (task && task.type === 'POSE') {
        taskPoints = [];
    }
    taskPoints.push({x, y});
    updateTaskForm();
}

$('complex-action').addEventListener('change', updateTaskForm);
$('task-pick').addEventListener('click', () => setPickMode(pickMode === 'waypoints' ? null : 'waypoints'));
$('task-clear').addEventListener('click', () => {
    taskPoints = [];
    taskTwists = [];
    updateTaskForm();
});
$('task-add-twist').addEventListener('click', () => {
    taskTwists.push({
        linear: {x: Number($('task-vx').value), y: Number($('task-vy').value), z: 0},
        angular: {x: 0, y: 0, z: Number($('task-wz').value)},
    });
    updateTaskForm();
});
$('task-send').addEventListener('click', async () => {
    const task = selectedTask();
    if (!task) {
        return;
    }
    const command = {name: task.name, type: task.type};
    if (POSE_TASKS.includes(task.type)) {
        if (!taskPoints.length) {
            toast('pick points in the 3D view first');
            return;
        }
        command.poses = taskPoints.map((p) => poseMessage(p.x, p.y, null));
    } else if (TWIST_TASKS.includes(task.type)) {
        if (!taskTwists.length) {
            toast('add twists first');
            return;
        }
        command.twists = taskTwists;
    }
    try {
        await send('COMPLEX_ACTION_COMMAND', command);
    } catch (error) {
        return;
    }
    setPickMode(null);
    taskPoints = [];
    taskTwists = [];
    updateTaskForm();
});

// ------------------------------------------------------------------ maps

function setMaps() {
    const mapType = TELEMETRY_TYPES.indexOf('MAP');
    const channels = state.channels
        .filter((channel) => (channel.messagetype || 0) === mapType)
        .map((channel) => ({name: channel.name, channel: channel.channelno || 0}));
    if (!channels.some((channel) => channel.channel === 0)) {
        channels.unshift({name: 'default', channel: 0});
    }
    $('maps').replaceChildren(...channels.map(({name, channel}) => {
        const description = element('span', {class: 'description muted'}, 'not loaded');
        const visible = element('input', {type: 'checkbox', checked: '', title: 'visible', class: 'hidden'});
        visible.addEventListener('change', () => viewer.setMapVisible(`map:${channel}`, visible.checked));
        const load = element('button', {}, 'Load');
        load.addEventListener('click', async () => {
            description.textContent = 'loading…';
            const reply = await request('MAP', channel, 60000);
            if (!reply || !reply.map) {
                description.textContent = 'no map';
                return;
            }
            try {
                description.textContent = viewer.setMap(`map:${channel}`, reply.map);
                visible.classList.remove('hidden');
                visible.checked = true;
            } catch (error) {
                description.textContent = error.message;
            }
        });
        return element('div', {class: 'map'}, element('strong', {}, name || `channel ${channel}`), description, visible, load);
    }));
}

// ------------------------------------------------------------------ log

const MAX_LOG_ENTRIES = 300;

rrc.on('LOG_MESSAGE', (log) => {
    const level = LOG_LEVELS[log.level || 0] || ((log.level || 0) >= 20 ? 'CUSTOM' : String(log.level));
    const entry = element('div', {class: 'log-entry'},
        element('span', {class: 'time'}, timeOf(log.header)),
        element('span', {class: `level level-${level}`}, level),
        element('span', {}, (log.message || '').trim()));
    const container = $('log');
    container.prepend(entry);
    while (container.childElementCount > MAX_LOG_ENTRIES) {
        container.lastChild.remove();
    }
});

$('log-clear').addEventListener('click', () => $('log').replaceChildren());
$('log-level').addEventListener('change', (event) => {
    send('LOG_LEVEL_SELECT', {level: event.target.value}).catch(() => {});
});

// ------------------------------------------------------------------ permission requests

const permissionRequests = [];

function showPermissionRequest() {
    const pending = permissionRequests[0];
    $('permission-dialog').classList.toggle('hidden', !pending);
    if (pending) {
        $('permission-description').textContent = pending.description || pending.requestuid;
    }
}

async function answerPermissionRequest(granted) {
    const pending = permissionRequests.shift();
    showPermissionRequest();
    if (pending) {
        await send('PERMISSION', {requestuid: pending.requestuid, granted}).catch(() => {});
    }
}

rrc.on('PERMISSION_REQUEST', (permissionRequest) => {
    if (!permissionRequests.some((pending) => pending.requestuid === permissionRequest.requestuid)) {
        permissionRequests.push(permissionRequest);
        showPermissionRequest();
    }
});

$('permission-grant').addEventListener('click', () => answerPermissionRequest(true));
$('permission-deny').addEventListener('click', () => answerPermissionRequest(false));

// ------------------------------------------------------------------ telemetry inspector

const telemetryStats = new Map();
let selectedTelemetry = null;

rrc.onAny((type, data, channel) => {
    const key = `${type}:${channel}`;
    let stats = telemetryStats.get(key);
    if (!stats) {
        stats = {type, channel, rate: 0, last: 0};
        telemetryStats.set(key, stats);
    }
    const now = performance.now();
    if (stats.last) {
        const rate = 1000 / Math.max(now - stats.last, 1);
        stats.rate = stats.rate ? stats.rate * 0.8 + rate * 0.2 : rate;
    }
    stats.last = now;
    stats.data = data;
});

/**
 * shortens large arrays and strings (images, point clouds) for display
 */
function summarize(value) {
    if (Array.isArray(value)) {
        const items = value.slice(0, 10).map(summarize);
        return value.length > 10 ? [...items, `… ${value.length - 10} more`] : items;
    }
    if (typeof value === 'string' && value.length > 120) {
        return `${value.slice(0, 60)}… (${value.length} characters)`;
    }
    if (value && typeof value === 'object') {
        return Object.fromEntries(Object.entries(value).map(([key, item]) => [key, summarize(item)]));
    }
    return value;
}

function renderTelemetry() {
    if (!$('telemetry-table').closest('details').open) {
        return;
    }
    const now = performance.now();
    const rows = Array.from(telemetryStats.entries())
        .sort(([a], [b]) => a.localeCompare(b))
        .map(([key, stats]) => {
            const age = (now - stats.last) / 1000;
            const row = element('tr', {class: key === selectedTelemetry ? 'selected' : ''},
                element('td', {}, stats.type.toLowerCase()),
                element('td', {}, stats.channel),
                element('td', {}, age < 3 ? format(stats.rate, 1) : '0'),
                element('td', {}, age < 60 ? `${format(age, 1)}s` : '>1m'));
            row.addEventListener('click', () => {
                selectedTelemetry = selectedTelemetry === key ? null : key;
                renderTelemetry();
            });
            return row;
        });
    $('telemetry-table').replaceChildren(...rows);
    const selected = telemetryStats.get(selectedTelemetry);
    $('telemetry-detail').textContent = selected ? JSON.stringify(summarize(selected.data), null, 2) : '';
}

setInterval(renderTelemetry, 1000);

// ------------------------------------------------------------------ robot info, files

const info = {};
const INFO_LABELS = {
    name: 'name', model: 'model', library: 'library version', protocol: 'protocol', git: 'git version',
    options: 'options', frames: 'controllable', commands: 'commands', telemetry: 'telemetry',
};

function setInfo(key, value) {
    info[key] = value;
    $('robot-info').replaceChildren(...Object.keys(INFO_LABELS).filter((k) => info[k] !== undefined).flatMap((k) =>
        [element('dt', {}, INFO_LABELS[k]), element('dd', {}, info[k])]));
}

function setFiles(definition) {
    const files = (definition && definition.file) || [];
    $('files').replaceChildren(...files.map((file, i) => {
        const links = element('div');
        const button = element('button', {}, `Download ${file.identifier}${(definition.isFolder || [])[i] ? '/' : ''}`);
        button.addEventListener('click', async () => {
            button.disabled = true;
            try {
                const folder = JSON.parse(await send('FILE_REQUEST', {identifier: file.identifier, compressed: false}));
                links.replaceChildren(...(folder.file || []).filter((f) => f.data).map((f) => {
                    const url = URL.createObjectURL(new Blob([base64ToBytes(f.data)]));
                    return element('a', {href: url, download: f.path.split('/').pop()}, f.path);
                }));
            } catch (error) {
                links.textContent = 'download failed';
            } finally {
                button.disabled = false;
            }
        });
        return element('div', {class: 'row'}, button, links);
    }));
}

$('reload-model').addEventListener('click', loadModel);
$('reload-interface').addEventListener('click', loadInterface);

// ------------------------------------------------------------------ cards: remember open state

document.querySelectorAll('details[data-card]').forEach((card) => {
    const key = `rrc-card-${card.dataset.card}`;
    try {
        const stored = localStorage.getItem(key);
        if (stored !== null) {
            card.open = stored === 'open';
        }
    } catch (error) {
        // storage may be unavailable
    }
    card.addEventListener('toggle', () => {
        try {
            localStorage.setItem(key, card.open ? 'open' : 'closed');
        } catch (error) {
            // storage may be unavailable
        }
        if (card.open && card.dataset.card === 'camera') {
            drawSelectedImage();
        }
    });
});

// ------------------------------------------------------------------ start

rrc.connectTelemetry();
rrc.startHeartbeat();
loadInterface();
