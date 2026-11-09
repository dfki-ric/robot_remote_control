import type { ImageMsg } from '../rrc/protocol';
import { base64ToBytes, format } from '../rrc/util';

const PIXEL_FORMATS: Record<string, [channels: number, order: [number, number, number]]> = {
	MODE_RGB: [3, [0, 1, 2]], RGB8: [3, [0, 1, 2]],
	MODE_BGR: [3, [2, 1, 0]], BGR8: [3, [2, 1, 0]],
	MODE_RGB32: [4, [0, 1, 2]], RGBA8: [4, [0, 1, 2]], BGRA8: [4, [2, 1, 0]],
	MODE_GRAYSCALE: [1, [0, 0, 0]], MONO8: [1, [0, 0, 0]]
};

const DEPTH_FORMATS = ['32FC1', 'MONO16', '16UC1'];

/**
 * draws a robot_remote_control::Image into the canvas, depth images are scaled to their value range
 * @returns a description of the image
 */
export async function drawImage(canvas: HTMLCanvasElement, image: ImageMsg): Promise<string> {
	const context = canvas.getContext('2d');
	if (!context) {
		throw new Error('no 2d canvas context');
	}
	const encoding = (image.encoding ?? '').toUpperCase();
	const bytes = base64ToBytes(image.data);
	const width = image.width ?? 0;
	const height = image.height ?? 0;
	let details = `${width}×${height} ${image.encoding ?? ''}`;

	if (/JPEG|JPG|PNG/.test(encoding)) {
		const bitmap = await createImageBitmap(new Blob([bytes]));
		canvas.width = bitmap.width;
		canvas.height = bitmap.height;
		context.drawImage(bitmap, 0, 0);
		return `${bitmap.width}×${bitmap.height} ${image.encoding}`;
	}
	if (!width || !height) {
		throw new Error('empty image');
	}
	const pixels = context.createImageData(width, height);
	const out = pixels.data;
	const pixelFormat = PIXEL_FORMATS[encoding];
	if (pixelFormat) {
		const [channels, order] = pixelFormat;
		const step = image.step || width * channels;
		for (let y = 0; y < height; ++y) {
			for (let x = 0; x < width; ++x) {
				const source = y * step + x * channels;
				const target = (y * width + x) * 4;
				out[target] = bytes[source + order[0]];
				out[target + 1] = bytes[source + order[1]];
				out[target + 2] = bytes[source + order[2]];
				out[target + 3] = 255;
			}
		}
	} else if (DEPTH_FORMATS.includes(encoding)) {
		const view = new DataView(bytes.buffer);
		const littleEndian = !image.isBigendian;
		const float = encoding === '32FC1';
		const size = float ? 4 : 2;
		const step = image.step || width * size;
		const values = new Float32Array(width * height);
		let min = Infinity;
		let max = -Infinity;
		for (let y = 0; y < height; ++y) {
			for (let x = 0; x < width; ++x) {
				const offset = y * step + x * size;
				const v = float ? view.getFloat32(offset, littleEndian) : view.getUint16(offset, littleEndian);
				values[y * width + x] = v;
				if (Number.isFinite(v) && v > 0) {
					min = Math.min(min, v);
					max = Math.max(max, v);
				}
			}
		}
		const range = max > min ? max - min : 1;
		values.forEach((v, i) => {
			const shade = Number.isFinite(v) && v > 0 ? 255 - (225 * (v - min)) / range : 0;
			out.set([shade, shade, shade, 255], i * 4);
		});
		details += `  ${format(min)}…${format(max)}`;
	} else {
		throw new Error(`unsupported image encoding ${image.encoding}`);
	}
	canvas.width = width;
	canvas.height = height;
	context.putImageData(pixels, 0, 0);
	return details;
}
