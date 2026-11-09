<script lang="ts">
	import RobotConsole from '#lib/components/RobotConsole.svelte';
	import { readConnectionConfig, type ConnectionConfig } from '#lib/config.ts';

	// the connection is fixed for the lifetime of the page, invalid url parameters stop here
	function readConfig(): { config: ConnectionConfig | null; error: string } {
		try {
			return { config: readConnectionConfig(window.location, import.meta.env.DEV), error: '' };
		} catch (e) {
			return { config: null, error: (e as Error).message };
		}
	}

	const { config, error } = readConfig();
</script>

{#if config}
	<RobotConsole {config} />
{:else}
	<main class="config-error">
		<h1>The interface cannot connect</h1>
		<p>{error}</p>
		<p>Open the page without the parameter or fix it, e.g. <code>?telemetryport=7002</code>.</p>
	</main>
{/if}

<style>
	.config-error {
		max-width: 60ch;
		margin: 15vh auto;
		padding: 0 24px;
	}

	code {
		color: var(--np-color-primary);
	}
</style>
