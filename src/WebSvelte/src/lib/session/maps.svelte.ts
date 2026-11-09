import type { GridMapMsg, MapContent, MapMsg, PointCloudMsg } from '../rrc/protocol';

export type MapKind = 'pointcloud' | 'gridmap';

export interface MapLayer {
	kind: MapKind;
	content: MapContent;
	visible: boolean;
}

export interface MapStatus {
	text: string;
	layer: MapLayer | null;
}

/**
 * @returns the kind of a robot_remote_control::Map content, throws for unsupported types
 */
export function mapKind(content: MapContent): MapKind {
	const type = (content['@type'] ?? '').split('.').pop();
	if (type === 'PointCloud') {
		return 'pointcloud';
	}
	if (type === 'GridMap' && (content as GridMapMsg).layers?.length) {
		return 'gridmap';
	}
	throw new Error(`unsupported map type ${content['@type'] ?? 'unknown'}`);
}

export function describeMap(content: MapContent): string {
	if (mapKind(content) === 'pointcloud') {
		return `point cloud, ${(content as PointCloudMsg).points?.length ?? 0} points`;
	}
	const layer = (content as GridMapMsg).layers?.[0];
	return `grid map ${layer?.name ?? ''} ${layer?.size?.x ?? 0}×${layer?.size?.y ?? 0}`;
}

/**
 * maps requested from the robot (they are large, so only on demand), by channel
 */
export class MapLayers {
	/** raw state: the map contents are large, the record is replaced on every change */
	status = $state.raw<Readonly<Record<number, MapStatus>>>({});

	constructor(private readonly request: (channel: number) => Promise<MapMsg | null>) {}

	private set(channel: number, status: MapStatus) {
		this.status = { ...this.status, [channel]: status };
	}

	async load(channel: number) {
		this.set(channel, { text: 'loading…', layer: null });
		const reply = await this.request(channel);
		if (!reply?.map) {
			this.set(channel, { text: 'no map', layer: null });
			return;
		}
		this.receive(channel, reply.map);
	}

	/**
	 * a map of a channel, requested or sent as telemetry, replaces the shown one
	 */
	receive(channel: number, content: MapContent) {
		try {
			const visible = this.status[channel]?.layer?.visible ?? true;
			this.set(channel, { text: describeMap(content), layer: { kind: mapKind(content), content, visible } });
		} catch (error) {
			this.set(channel, { text: (error as Error).message, layer: null });
		}
	}

	setVisible(channel: number, visible: boolean) {
		const status = this.status[channel];
		if (status?.layer) {
			this.set(channel, { ...status, layer: { ...status.layer, visible } });
		}
	}
}
