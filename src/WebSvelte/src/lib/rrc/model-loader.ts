/**
 * Loads the urdf model of a robot:
 * - hosted by the http server (e.g. robot_remote_control-web_proxy caches the model of the robot): the files
 *   are at /model/<folder name>/, listed in /model/<folder name>/model.json
 *   (`{path, modelfilename, files: [paths relative to the folder]}`)
 * - otherwise downloaded through robot_remote_control: the model folder is transferred with a FILE_REQUEST
 *   (like RobotController::requestRobotModel())
 * Mesh urls in the urdf (package://, file://, relative) are mapped to the model files by the longest matching
 * path suffix, the result is parsed with the UrdfParser of urdf-viewer.
 */
import { UrdfParser, type IUrdfMesh, type IUrdfRobot, type IUrdfVisual } from 'urdf-viewer';
import type { RobotRemoteControl } from './client';
import type { FolderMsg, RobotModelInformationMsg } from './protocol';
import { base64ToBytes } from './util';

const MIME_TYPES: Record<string, string> = {
	stl: 'model/stl', dae: 'model/vnd.collada+xml', obj: 'model/obj', png: 'image/png',
	jpg: 'image/jpeg', jpeg: 'image/jpeg', tga: 'image/x-tga', urdf: 'application/xml'
};

/**
 * @returns the path segments without schemes, empty and "." segments, ".." resolved
 */
export function pathSegments(path: string): string[] {
	const segments: string[] = [];
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
 * the folder name of a robot on the server, the same rule as robot_remote_control-web_proxy
 */
export function modelFolderName(robotName: string): string {
	// one _ per character (code point), like the proxy does for utf-8
	const name = robotName.replace(/[^A-Za-z0-9_.-]/gu, '_');
	return ['', '.', '..'].includes(name) ? 'robot' : name;
}

interface ModelFile {
	path: string;
	url: string;
	segments: string[];
	bytes?: Uint8Array<ArrayBuffer>;
}

/**
 * the files of a model, paths are resolved by the longest matching path suffix
 */
export class ModelFiles {
	private readonly files: ModelFile[];

	constructor(files: Omit<ModelFile, 'segments'>[]) {
		this.files = files.map((file) => ({ ...file, segments: pathSegments(file.path) }));
	}

	find(path: string): ModelFile | null {
		const wanted = pathSegments(path);
		let best: ModelFile | null = null;
		let bestMatch = 0;
		for (const file of this.files) {
			let match = 0;
			while (
				match < wanted.length &&
				match < file.segments.length &&
				wanted[wanted.length - 1 - match] === file.segments[file.segments.length - 1 - match]
			) {
				++match;
			}
			if (match > bestMatch) {
				best = file;
				bestMatch = match;
			}
		}
		return best;
	}

	dispose() {
		this.files.filter((file) => file.url.startsWith('blob:')).forEach((file) => URL.revokeObjectURL(file.url));
	}
}

interface ModelSource {
	files: ModelFiles;
	urdf: ModelFile;
	text: string;
}

async function hostedModel(robotName: string): Promise<ModelSource | null> {
	const base = `/model/${modelFolderName(robotName)}`;
	let index: { path?: string; modelfilename?: string; files?: string[] } | null;
	try {
		const response = await fetch(`${base}/model.json`, { cache: 'no-cache' });
		index = response.ok ? await response.json() : null;
	} catch {
		return null;
	}
	if (!index || !Array.isArray(index.files) || !index.path) {
		return null;
	}
	const files = new ModelFiles(
		index.files.map((path) => ({
			path,
			url: `${base}/${pathSegments(path).map(encodeURIComponent).join('/')}`
		}))
	);
	const urdf = files.find(index.modelfilename ? `${index.path}/${index.modelfilename}` : index.path);
	if (!urdf) {
		return null;
	}
	const response = await fetch(urdf.url);
	return response.ok ? { files, urdf, text: await response.text() } : null;
}

async function transferredModel(rrc: RobotRemoteControl, onStatus: (text: string) => void): Promise<ModelSource | null> {
	onStatus('requesting the model information');
	const info = await rrc.requestTelemetry<RobotModelInformationMsg>('ROBOT_MODEL_INFORMATION');
	const definition = info?.filedef?.file?.[0];
	if (!definition?.identifier || !definition.path) {
		return null;
	}
	onStatus('downloading the model');
	const folder = await rrc.commandJson<FolderMsg>('FILE_REQUEST', { identifier: definition.identifier, compressed: false }, 120000);
	const transfers = (folder?.file ?? []).filter((file) => file.data && file.path);
	if (!transfers.length) {
		throw new Error(`the model files of '${definition.identifier}' could not be downloaded`);
	}
	const files = new ModelFiles(
		transfers.map((transfer) => {
			const path = transfer.path as string;
			const bytes = base64ToBytes(transfer.data);
			const extension = path.split('.').pop()?.toLowerCase() ?? '';
			const blob = new Blob([bytes], { type: MIME_TYPES[extension] ?? 'application/octet-stream' });
			return { path, bytes, url: URL.createObjectURL(blob) };
		})
	);
	// folder model: path is the folder, modelfilename the urdf file in it
	const urdfPath = info?.modelfilename ? `${definition.path}/${info.modelfilename}` : definition.path;
	const urdf = files.find(urdfPath);
	if (!urdf?.bytes) {
		files.dispose();
		throw new Error(`the model file ${urdfPath} is missing in the download`);
	}
	return { files, urdf, text: new TextDecoder().decode(urdf.bytes) };
}

export interface LoadedModel {
	robot: IUrdfRobot;
	files: ModelFiles;
	/** mesh files referenced by the urdf that are not part of the model, their visuals are removed */
	missing: string[];
	source: 'server' | 'robot';
}

/**
 * Prepares the visuals for urdf-viewer: fills in the default origin (UrdfParser leaves it undefined when the
 * <origin> element or one of its attributes is missing, the viewer then fails), points meshes to the model files
 * and removes visuals whose file is missing.
 * @returns the missing mesh files
 */
function prepareVisuals(robot: IUrdfRobot, files: ModelFiles): string[] {
	const missing = new Set<string>();
	const resolve = (visuals: IUrdfVisual[]) =>
		visuals.filter((visual) => {
			visual.origin_xyz ??= [0, 0, 0];
			visual.origin_rpy ??= [0, 0, 0];
			if (visual.type !== 'mesh') {
				return true;
			}
			const mesh = visual.geometry as IUrdfMesh;
			const file = files.find(mesh.filename);
			if (!file) {
				missing.add(mesh.filename);
				return false;
			}
			mesh.filename = file.url;
			return true;
		});
	for (const link of Object.values(robot.links)) {
		link.visual = resolve(link.visual);
		link.collision = resolve(link.collision);
	}
	return [...missing];
}

/**
 * @param robotName name of the robot, to find a hosted model
 * @param onStatus called with progress messages
 * @returns null if the robot has no model
 */
export async function loadRobotModel(
	rrc: RobotRemoteControl,
	robotName: string,
	onStatus: (text: string) => void = () => {}
): Promise<LoadedModel | null> {
	onStatus('looking for the model on the server');
	let model = await hostedModel(robotName);
	const source = model ? 'server' : 'robot';
	model ??= await transferredModel(rrc, onStatus);
	if (!model) {
		return null;
	}
	onStatus('parsing the model');
	try {
		// relative mesh paths are resolved against the urdf path, package:// becomes /<package>/...
		const parser = new UrdfParser(model.urdf.segments.join('/'));
		const robot = parser.fromString(model.text);
		const missing = prepareVisuals(robot, model.files);
		return { robot, files: model.files, missing, source };
	} catch (error) {
		model.files.dispose();
		throw error;
	}
}
