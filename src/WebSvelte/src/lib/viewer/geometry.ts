/**
 * Conversions of telemetry into three.js objects, in robot coordinates (z up).
 */
import { DataTexture, Group, Mesh, MeshBasicMaterial, NearestFilter, PlaneGeometry } from 'three';
import type { GridMapMsg, LaserScanMsg, PointCloudMsg, PosesMsg } from '../rrc/protocol';
import { toQuaternion, toVector3 } from './frames.svelte';

export function pathPositions(path: PosesMsg | null): Float32Array {
	const poses = path?.poses ?? [];
	const positions = new Float32Array(poses.length * 3);
	poses.forEach((pose, i) => {
		positions.set([pose.position?.x ?? 0, pose.position?.y ?? 0, (pose.position?.z ?? 0) + 0.02], i * 3);
	});
	return positions;
}

export function laserPositions(scan: LaserScanMsg | null): Float32Array {
	const positions: number[] = [];
	const ranges = scan?.ranges ?? [];
	const start = scan?.startAngle ?? 0;
	const step = scan?.horizontalAngleResolution ?? 0;
	const min = scan?.rangeMin ?? 0;
	const max = scan?.rangeMax || Infinity;
	ranges.forEach((range, i) => {
		if (range > min && range < max) {
			const angle = start + i * step;
			positions.push(range * Math.cos(angle), range * Math.sin(angle), 0);
		}
	});
	return new Float32Array(positions);
}

export function cloudPositions(cloud: PointCloudMsg | null): Float32Array {
	const points = cloud?.points ?? [];
	const positions = new Float32Array(points.length * 3);
	points.forEach((p, i) => positions.set([p.x ?? 0, p.y ?? 0, p.z ?? 0], i * 3));
	return positions;
}

/**
 * the first layer of a grid map as a textured plane, value[x + y * sizeX], the origin is the grid corner
 */
export function gridMapObject(map: GridMapMsg): Group {
	const layer = map.layers?.[0];
	if (!layer) {
		throw new Error('grid map without layers');
	}
	const sizeX = layer.size?.x || 1;
	const sizeY = layer.size?.y || 1;
	const scaleX = layer.scale?.x || 1;
	const scaleY = layer.scale?.y || 1;
	const values = layer.value ?? [];
	let min = Infinity;
	let max = -Infinity;
	for (const v of values) {
		if (Number.isFinite(v)) {
			min = Math.min(min, v);
			max = Math.max(max, v);
		}
	}
	const range = max > min ? max - min : 1;
	const data = new Uint8Array(sizeX * sizeY * 4);
	for (let i = 0; i < sizeX * sizeY; ++i) {
		const v = values[i];
		const finite = Number.isFinite(v);
		const shade = finite ? 40 + (200 * (v - min)) / range : 0;
		data.set([shade, shade, shade + 15, finite ? 170 : 0], i * 4);
	}
	const texture = new DataTexture(data, sizeX, sizeY);
	texture.needsUpdate = true;
	texture.magFilter = NearestFilter;
	const plane = new Mesh(
		new PlaneGeometry(sizeX * scaleX, sizeY * scaleY),
		new MeshBasicMaterial({ map: texture, transparent: true, depthWrite: false })
	);
	plane.position.set((sizeX * scaleX) / 2, (sizeY * scaleY) / 2, 0.005);
	const group = new Group();
	group.add(plane);
	group.position.copy(toVector3(map.origin?.position));
	group.quaternion.copy(toQuaternion(map.origin?.orientation));
	return group;
}

export function disposeObject(group: Group) {
	group.traverse((object) => {
		if (object instanceof Mesh) {
			object.geometry.dispose();
			const material = object.material as MeshBasicMaterial;
			material.map?.dispose();
			material.dispose();
		}
	});
}
