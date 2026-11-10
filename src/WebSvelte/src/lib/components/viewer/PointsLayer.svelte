<script lang="ts">
	import { T } from '@threlte/core';
	import { BufferGeometry, Float32BufferAttribute } from 'three';

	interface Props {
		/** x, y, z of the points */
		positions: Float32Array;
		color: number;
		/** point size in m */
		size: number;
		visible?: boolean;
	}

	let { positions, color, size, visible = true }: Props = $props();

	const geometry = new BufferGeometry();

	$effect(() => {
		geometry.setAttribute('position', new Float32BufferAttribute(positions, 3));
		geometry.computeBoundingSphere();
	});

	$effect(() => () => geometry.dispose());
</script>

<T.Points {geometry} {visible} frustumCulled={false}>
	<T.PointsMaterial {color} {size} sizeAttenuation />
</T.Points>
