<script lang="ts">
	import { T } from '@threlte/core';
	import { BufferGeometry, Float32BufferAttribute } from 'three';

	interface Props {
		/** x, y, z of the line points */
		positions: Float32Array;
		color: number;
		visible?: boolean;
	}

	let { positions, color, visible = true }: Props = $props();

	const geometry = new BufferGeometry();

	$effect(() => {
		geometry.setAttribute('position', new Float32BufferAttribute(positions, 3));
		geometry.computeBoundingSphere();
	});

	$effect(() => () => geometry.dispose());
</script>

<T.Line {geometry} {visible} frustumCulled={false}>
	<T.LineBasicMaterial {color} />
</T.Line>
