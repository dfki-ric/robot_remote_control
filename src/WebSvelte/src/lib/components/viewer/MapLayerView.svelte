<script lang="ts">
	import { T } from '@threlte/core';
	import type { GridMapMsg, PointCloudMsg } from '#lib/rrc/protocol.ts';
	import type { MapLayer } from '#lib/session/maps.svelte.ts';
	import { toQuaternion, toVector3 } from '#lib/viewer/frames.svelte.ts';
	import { cloudPositions, disposeObject, gridMapObject } from '#lib/viewer/geometry.ts';
	import FrameGroup from './FrameGroup.svelte';
	import PointsLayer from './PointsLayer.svelte';

	let { layer }: { layer: MapLayer } = $props();

	const cloud = $derived(layer.kind === 'pointcloud' ? (layer.content as PointCloudMsg) : null);
	const grid = $derived(layer.kind === 'gridmap' ? (layer.content as GridMapMsg) : null);
	const gridObject = $derived(grid ? gridMapObject(grid) : null);

	$effect(() => {
		const object = gridObject;
		return () => {
			if (object) {
				disposeObject(object);
			}
		};
	});
</script>

{#if cloud}
	<FrameGroup frame={cloud.header?.frame}>
		<T.Group position={toVector3(cloud.origin?.position).toArray()} quaternion={toQuaternion(cloud.origin?.orientation).toArray()}>
			<PointsLayer positions={cloudPositions(cloud)} color={0x9ccc65} size={0.04} visible={layer.visible} />
		</T.Group>
	</FrameGroup>
{:else if grid && gridObject}
	<FrameGroup frame={grid.frame || grid.header?.frame}>
		<T is={gridObject} visible={layer.visible} />
	</FrameGroup>
{/if}
