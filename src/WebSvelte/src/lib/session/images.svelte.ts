import type { ImageMsg } from '../rrc/protocol';
import { RateMeter } from '../rrc/util';

export interface ImageSourceInfo {
	/** "image:<channel>" or "layers:<channel>:<layer>" */
	key: string;
	label: string;
}

interface ImageSource extends ImageSourceInfo {
	image: ImageMsg;
	rate: RateMeter;
}

/**
 * The latest image of every image channel and image layer. Only the list of sources is reactive,
 * views subscribe to the (frequent) image updates and draw them into a canvas.
 */
export class ImageSources {
	list = $state.raw<ImageSourceInfo[]>([]);
	private readonly sources = new Map<string, ImageSource>();
	private readonly listeners = new Set<(key: string) => void>();

	update(key: string, label: string, image: ImageMsg) {
		let source = this.sources.get(key);
		if (!source) {
			source = { key, label, image, rate: new RateMeter() };
			this.sources.set(key, source);
			this.list = [...this.list, { key, label }];
		} else if (source.label !== label) {
			source.label = label;
			this.list = this.list.map((info) => (info.key === key ? { key, label } : info));
		}
		source.image = image;
		source.rate.tick();
		this.listeners.forEach((listener) => listener(key));
	}

	get(key: string): { image: ImageMsg; rate: number } | null {
		const source = this.sources.get(key);
		return source ? { image: source.image, rate: source.rate.rate } : null;
	}

	label(key: string): string | null {
		return this.sources.get(key)?.label ?? null;
	}

	/**
	 * @returns function to remove the listener
	 */
	subscribe(listener: (key: string) => void): () => void {
		this.listeners.add(listener);
		return () => this.listeners.delete(listener);
	}
}
