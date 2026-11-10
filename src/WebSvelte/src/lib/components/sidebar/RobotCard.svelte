<script lang="ts">
	import { Button } from 'noph-ui';
	import { getSession, type DownloadedFile } from '#lib/session/session.svelte.ts';
	import SidebarCard from './SidebarCard.svelte';

	const session = getSession();

	const files = $derived(
		(session.files?.file ?? []).map((file, i) => ({
			identifier: file.identifier ?? '',
			folder: Boolean(session.files?.isFolder?.[i])
		}))
	);

	let downloads = $state<Record<string, DownloadedFile[] | 'busy' | 'failed'>>({});

	async function download(identifier: string) {
		const previous = downloads[identifier];
		if (Array.isArray(previous)) {
			previous.forEach((file) => URL.revokeObjectURL(file.url));
		}
		downloads[identifier] = 'busy';
		try {
			downloads[identifier] = await session.downloadFiles(identifier);
		} catch {
			downloads[identifier] = 'failed';
		}
	}
</script>

<SidebarCard id="info" title="Robot">
	<dl>
		{#each session.infoRows as [label, value] (label)}
			<dt>{label}</dt>
			<dd>{value}</dd>
		{/each}
	</dl>
	<div class="row">
		<Button size="xs" variant="tonal" onclick={() => session.loadModel()}>Reload model</Button>
		<Button size="xs" variant="tonal" onclick={() => session.loadInterface()}>Reload interface</Button>
	</div>
	{#each files as file (file.identifier)}
		{@const state = downloads[file.identifier]}
		<div class="row">
			<Button size="xs" variant="outlined" disabled={state === 'busy'} onclick={() => download(file.identifier)}>
				Download {file.identifier}{file.folder ? '/' : ''}
			</Button>
			{#if state === 'failed'}
				<span class="small">The download failed.</span>
			{:else if Array.isArray(state)}
				{#each state as item (item.url)}
					<a class="small" href={item.url} download={item.path.split('/').pop()}>{item.path}</a>
				{/each}
			{/if}
		</div>
	{/each}
</SidebarCard>

<style>
	dl {
		display: grid;
		grid-template-columns: auto 1fr;
		gap: 4px 12px;
		margin: 0;
		font-size: 0.8125rem;
	}

	dt {
		color: var(--np-color-on-surface-variant);
	}

	dd {
		margin: 0;
		overflow-wrap: anywhere;
	}

	a {
		color: var(--np-color-primary);
	}
</style>
