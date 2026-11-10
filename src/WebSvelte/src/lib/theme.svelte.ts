/**
 * Theme of noph-ui: color scheme (system, light, dark) and an optional palette generated from a source
 * color with the Material color utilities (the same roles noph-ui's default theme defines). Remembered by
 * the browser, previewed while the settings are open.
 */
import {
	Hct, MaterialDynamicColors, SchemeContent, SchemeExpressive, SchemeFidelity, SchemeMonochrome, SchemeNeutral,
	SchemeTonalSpot, SchemeVibrant, argbFromHex, hexFromArgb, type DynamicScheme
} from '@materialx/material-color-utilities';
import { createContext } from 'svelte';
import { storageGetJson, storageSetJson } from './storage';

export type ThemeMode = 'system' | 'light' | 'dark';
export type ThemeVariant = 'default' | 'tonalSpot' | 'vibrant' | 'expressive' | 'fidelity' | 'content' | 'neutral' | 'monochrome';

export interface ThemeConfig {
	mode: ThemeMode;
	/** 'default' keeps the colors of noph-ui's default theme */
	variant: ThemeVariant;
	/** source color of generated palettes, #rrggbb */
	source: string;
	/** -1 (reduced) to 1 (high) */
	contrast: number;
}

export const THEME_MODES: Record<ThemeMode, string> = { system: 'System', light: 'Light', dark: 'Dark' };

export const THEME_VARIANTS: Record<ThemeVariant, string> = {
	default: 'noph-ui default',
	tonalSpot: 'Tonal spot',
	vibrant: 'Vibrant',
	expressive: 'Expressive',
	fidelity: 'Fidelity',
	content: 'Content',
	neutral: 'Neutral',
	monochrome: 'Monochrome'
};

export const CONTRAST_LEVELS: Record<string, string> = { '0': 'Standard', '0.5': 'Medium', '1': 'High' };

export const SOURCE_PRESETS = ['#0980ad', '#2e7d32', '#00897b', '#c62828', '#ef6c00', '#6a1b9a', '#546e7a'];

type SchemeClass = new (source: Hct, isDark: boolean, contrastLevel: number) => DynamicScheme;

const SCHEMES: Record<Exclude<ThemeVariant, 'default'>, SchemeClass> = {
	tonalSpot: SchemeTonalSpot,
	vibrant: SchemeVibrant,
	expressive: SchemeExpressive,
	fidelity: SchemeFidelity,
	content: SchemeContent,
	neutral: SchemeNeutral,
	monochrome: SchemeMonochrome
};

/** the color roles of noph-ui's default theme, as MaterialDynamicColors method names */
const ROLES = [
	'background', 'error', 'errorContainer', 'inverseOnSurface', 'inversePrimary', 'inverseSurface',
	'neutralPaletteKeyColor', 'neutralVariantPaletteKeyColor', 'onBackground', 'onError', 'onErrorContainer',
	'onPrimary', 'onPrimaryContainer', 'onPrimaryFixed', 'onPrimaryFixedVariant', 'onSecondary',
	'onSecondaryContainer', 'onSecondaryFixed', 'onSecondaryFixedVariant', 'onSurface', 'onSurfaceVariant',
	'onTertiary', 'onTertiaryContainer', 'onTertiaryFixed', 'onTertiaryFixedVariant', 'outline', 'outlineVariant',
	'primary', 'primaryContainer', 'primaryFixed', 'primaryFixedDim', 'primaryPaletteKeyColor', 'scrim', 'secondary',
	'secondaryContainer', 'secondaryFixed', 'secondaryFixedDim', 'secondaryPaletteKeyColor', 'shadow', 'surface',
	'surfaceBright', 'surfaceContainer', 'surfaceContainerHigh', 'surfaceContainerHighest', 'surfaceContainerLow',
	'surfaceContainerLowest', 'surfaceDim', 'surfaceTint', 'surfaceVariant', 'tertiary', 'tertiaryContainer',
	'tertiaryFixed', 'tertiaryFixedDim', 'tertiaryPaletteKeyColor'
] as const;

const cssName = (role: string) => `--np-color-${role.replace(/[A-Z]/g, (c) => `-${c.toLowerCase()}`)}`;

export function defaultThemeConfig(): ThemeConfig {
	return { mode: 'dark', variant: 'default', source: SOURCE_PRESETS[0], contrast: 0 };
}

/**
 * @returns the stored theme if it is valid, null otherwise
 */
export function parseThemeConfig(value: unknown): ThemeConfig | null {
	const config = value as ThemeConfig;
	const valid =
		config !== null && typeof config === 'object' &&
		config.mode in THEME_MODES &&
		config.variant in THEME_VARIANTS &&
		typeof config.source === 'string' && /^#[0-9a-f]{6}$/i.test(config.source) &&
		typeof config.contrast === 'number' && config.contrast >= -1 && config.contrast <= 1;
	return valid ? config : null;
}

/**
 * @returns css variable -> light-dark(light, dark) color for generated palettes, empty for the default theme
 */
export function themeVariables(config: ThemeConfig): Map<string, string> {
	const variables = new Map<string, string>();
	if (config.variant === 'default') {
		return variables;
	}
	const Scheme = SCHEMES[config.variant];
	const source = Hct.fromInt(argbFromHex(config.source));
	const light = new Scheme(source, false, config.contrast);
	const dark = new Scheme(source, true, config.contrast);
	const colors = new MaterialDynamicColors();
	for (const role of ROLES) {
		const color = colors[role]();
		variables.set(cssName(role), `light-dark(${hexFromArgb(color.getArgb(light))}, ${hexFromArgb(color.getArgb(dark))})`);
	}
	return variables;
}

const STORAGE_KEY = 'theme';

export class ThemeSettings {
	/** the saved theme, the page may show a preview */
	config = $state<ThemeConfig>(defaultThemeConfig());

	constructor(private readonly root: HTMLElement = document.documentElement) {
		const stored = storageGetJson<unknown>(STORAGE_KEY);
		const parsed = stored === null ? null : parseThemeConfig(stored);
		if (stored !== null && parsed === null) {
			console.warn('the stored theme is invalid, using the default theme');
		}
		this.config = parsed ?? defaultThemeConfig();
		this.render(this.config);
	}

	/** shows a theme without saving it */
	preview(config: ThemeConfig) {
		this.render(config);
	}

	/** shows the saved theme again (after a preview) */
	restore() {
		this.render(this.config);
	}

	/**
	 * @throws for invalid themes, nothing is changed then
	 */
	apply(config: ThemeConfig) {
		if (!parseThemeConfig(config)) {
			throw new Error('invalid theme');
		}
		this.config = config;
		storageSetJson(STORAGE_KEY, config);
		this.render(config);
	}

	private render(config: ThemeConfig) {
		if (config.mode === 'system') {
			delete this.root.dataset.theme;
		} else {
			this.root.dataset.theme = config.mode;
		}
		// inline variables override noph-ui's default theme, removing them shows it again
		const variables = themeVariables(config);
		for (const role of ROLES) {
			const name = cssName(role);
			const value = variables.get(name);
			if (value) {
				this.root.style.setProperty(name, value);
			} else {
				this.root.style.removeProperty(name);
			}
		}
	}
}

export const [getThemeSettings, setThemeSettings] = createContext<ThemeSettings>();
