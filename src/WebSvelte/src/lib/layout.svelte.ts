/**
 * Layout of the page that is remembered by the browser: regions above (images) and below (log) the 3D view,
 * the image views and the open sidebar cards.
 */
import { createContext } from 'svelte';
import { storageGet, storageGetJson, storageSet, storageSetJson } from './storage';

export const MIN_REGION_HEIGHT = 80;
/** space kept for the 3D view when resizing regions */
export const MIN_STAGE_HEIGHT = 160;

export class RegionLayout {
	visible = $state(false);
	/** px, null for the default height */
	height = $state<number | null>(null);
	/** whether the user chose the visibility (it is not changed automatically then) */
	readonly visibilityRemembered: boolean;

	constructor(
		private readonly name: string,
		defaultVisible: boolean
	) {
		const stored = storageGet(`${name}-region`);
		this.visibilityRemembered = stored !== null;
		this.visible = stored === null ? defaultVisible : stored === 'shown';
		const height = Number(storageGet(`${name}-height`));
		this.height = height >= MIN_REGION_HEIGHT ? height : null;
	}

	setVisible(visible: boolean, remember = true) {
		this.visible = visible;
		if (remember) {
			storageSet(`${this.name}-region`, visible ? 'shown' : 'hidden');
		}
	}

	setHeight(height: number, remember: boolean) {
		this.height = Math.round(height);
		if (remember) {
			storageSet(`${this.name}-height`, String(this.height));
		}
	}
}

export interface ImageViewEntry {
	id: number;
	/** image source key, empty if none was chosen */
	key: string;
}

export class LayoutState {
	readonly images = new RegionLayout('image', false);
	readonly log = new RegionLayout('log', true);
	imageViews = $state<ImageViewEntry[]>([]);
	/** whether the user arranged the image views before (none are added automatically then) */
	readonly imageViewsRemembered: boolean;
	private viewId = 0;

	constructor() {
		const keys = storageGetJson<unknown>('image-views');
		this.imageViewsRemembered = Array.isArray(keys);
		if (Array.isArray(keys)) {
			this.imageViews = keys.map((key) => ({ id: ++this.viewId, key: String(key) }));
		}
	}

	private saveImageViews() {
		storageSetJson('image-views', this.imageViews.map((view) => view.key));
	}

	addImageView(key: string) {
		this.imageViews.push({ id: ++this.viewId, key });
		this.saveImageViews();
	}

	setImageView(id: number, key: string) {
		const view = this.imageViews.find((entry) => entry.id === id);
		if (view) {
			view.key = key;
			this.saveImageViews();
		}
	}

	removeImageView(id: number) {
		this.imageViews = this.imageViews.filter((view) => view.id !== id);
		this.saveImageViews();
	}

	cardOpen(card: string, defaultOpen: boolean): boolean {
		const stored = storageGet(`card-${card}`);
		return stored === null ? defaultOpen : stored === 'open';
	}

	setCardOpen(card: string, open: boolean) {
		storageSet(`card-${card}`, open ? 'open' : 'closed');
	}
}

export const [getLayout, setLayout] = createContext<LayoutState>();
