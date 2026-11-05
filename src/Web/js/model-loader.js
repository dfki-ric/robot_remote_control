/**
 * Loads the urdf model of a robot through robot_remote_control: the model folder is downloaded
 * with a FILE_REQUEST (like RobotController::requestRobotModel()), mesh and texture urls in the urdf
 * (package://, file://, relative) are mapped to the downloaded files by their path.
 */
import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import URDFLoader from '../lib/urdf-loader/src/URDFLoader.js';
import { base64ToBytes } from './rrc-client.js';

const MIME_TYPES = {
    stl: 'model/stl', dae: 'model/vnd.collada+xml', obj: 'model/obj', png: 'image/png',
    jpg: 'image/jpeg', jpeg: 'image/jpeg', tga: 'image/x-tga', urdf: 'application/xml',
};

/**
 * @returns the path segments without schemes, empty and "." segments, ".." resolved
 */
function pathSegments(path) {
    const segments = [];
    const withoutScheme = path.replace(/[?#].*$/, '').replace(/^([a-z][a-z0-9+.-]*:)+(\/\/)?/i, '');
    for (const segment of withoutScheme.split(/[\\/]+/)) {
        if (segment === '' || segment === '.' || /^[a-z][a-z0-9+.-]*:$/i.test(segment)) {
            continue;
        }
        if (segment === '..') {
            segments.pop();
        } else {
            segments.push(segment);
        }
    }
    return segments;
}

/**
 * the downloaded files, urls are resolved by the longest matching path suffix
 */
class DownloadedFiles {
    constructor(transfers) {
        this.files = transfers.map((transfer) => {
            const segments = pathSegments(transfer.path);
            const bytes = base64ToBytes(transfer.data);
            const extension = (segments[segments.length - 1] || '').split('.').pop().toLowerCase();
            const blob = new Blob([bytes], {type: MIME_TYPES[extension] || 'application/octet-stream'});
            return {path: transfer.path, segments, bytes, url: URL.createObjectURL(blob)};
        });
        this.urls = new Set(this.files.map((file) => file.url));
    }

    find(path) {
        const wanted = pathSegments(path);
        let best = null;
        let bestMatch = 0;
        for (const file of this.files) {
            let match = 0;
            while (match < wanted.length && match < file.segments.length &&
                   wanted[wanted.length - 1 - match] === file.segments[file.segments.length - 1 - match]) {
                ++match;
            }
            if (match > bestMatch) {
                best = file;
                bestMatch = match;
            }
        }
        return best;
    }

    resolve(url) {
        if (this.urls.has(url)) {
            return url;
        }
        const file = this.find(url);
        return file ? file.url : url;
    }

    dispose() {
        this.files.forEach((file) => URL.revokeObjectURL(file.url));
    }
}

/**
 * @param rrc RobotRemoteControl client
 * @param onStatus called with progress messages
 * @returns {robot: URDFRobot, files, missing: urls that could not be loaded} or null if the robot has no model
 */
export async function loadRobotModel(rrc, onStatus = () => {}) {
    onStatus('requesting the model information');
    const info = await rrc.requestTelemetry('ROBOT_MODEL_INFORMATION');
    const definition = info && info.filedef && info.filedef.file && info.filedef.file[0];
    if (!definition) {
        return null;
    }

    onStatus('downloading the model');
    const folder = await rrc.commandJson('FILE_REQUEST', {identifier: definition.identifier, compressed: false}, 120000);
    const transfers = ((folder && folder.file) || []).filter((file) => file.data);
    if (!transfers.length) {
        throw new Error(`the model files of '${definition.identifier}' could not be downloaded`);
    }
    const files = new DownloadedFiles(transfers);
    // folder model: path is the folder, modelfilename the urdf file in it
    const urdfPath = info.modelfilename ? `${definition.path}/${info.modelfilename}` : definition.path;
    const urdf = files.find(urdfPath);
    if (!urdf) {
        throw new Error(`the model file ${urdfPath} is missing in the download`);
    }

    onStatus('loading the model');
    const manager = new THREE.LoadingManager();
    manager.setURLModifier((url) => files.resolve(url));
    const missing = [];
    manager.onError = (url) => missing.push(url);
    let loading = false;
    manager.onStart = () => {
        loading = true;
    };
    const loaded = new Promise((resolve) => {
        manager.onLoad = resolve;
    });

    const loader = new URDFLoader(manager);
    // package://name/path is resolved to name/path, then mapped to a downloaded file
    loader.packages = (name) => name;
    loader.loadMeshCb = (path, meshManager, material, done) => {
        if (/\.obj$/i.test(path)) {
            new OBJLoader(meshManager).load(path, (object) => {
                if (material) {
                    object.traverse((child) => {
                        if (child.isMesh) {
                            child.material = material;
                        }
                    });
                }
                done(object);
            }, undefined, (error) => done(null, error));
        } else {
            loader.defaultMeshLoader(path, meshManager, material, done);
        }
    };
    const workingPath = urdf.segments.slice(0, -1).join('/') + '/';
    const robot = loader.parse(new TextDecoder().decode(urdf.bytes), workingPath);
    // meshes are loaded asynchronously, primitives only models load nothing
    if (loading) {
        await loaded;
    }
    return {robot, files, missing};
}
