<script lang="ts">
	// an icon of Google's Material Symbols (outlined) font, by its ligature name, e.g. "sports_esports"

	let { name, size = 24 }: { name: string; size?: number } = $props();
</script>

<span class="material-symbol" style:font-size="{size}px" aria-hidden="true">{name}</span>

<style>
	/* self-hosted (robots are often offline) and inlined into the page, see vite.config.ts */
	@font-face {
		font-family: 'Material Symbols Outlined';
		font-style: normal;
		font-weight: 400;
		/* block: never show the ligature name as text while the font loads */
		font-display: block;
		src: url('@fontsource/material-symbols-outlined/files/material-symbols-outlined-latin-400-normal.woff2') format('woff2');
	}

	.material-symbol {
		display: inline-block;
		width: 1em;
		height: 1em;
		overflow: hidden;
		font-family: 'Material Symbols Outlined';
		font-weight: normal;
		font-style: normal;
		line-height: 1;
		letter-spacing: normal;
		text-transform: none;
		white-space: nowrap;
		direction: ltr;
		font-feature-settings: 'liga';
		-webkit-font-smoothing: antialiased;
		user-select: none;
	}
</style>
