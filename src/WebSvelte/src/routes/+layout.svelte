<script lang="ts">
	import 'noph-ui/defaultTheme';
	import '../app.css';
	import favicon from '#lib/assets/favicon.svg';
	import type { LayoutProps } from './$types';

	let { children }: LayoutProps = $props();
</script>

<svelte:head>
	<link rel="icon" href={favicon} />
	<title>Robot Remote Control</title>
</svelte:head>

{@render children()}
