/**
 * 3D view of the robot: urdf model at the current pose with the current joint values, path,
 * laser scans, point clouds and maps. Picks goals and waypoints on the ground.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

const GROUND = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const WORLD_FRAMES = new Set(['', 'world', 'map', 'odom']);
const MAX_TRAIL_POINTS = 5000;

function vector(v) {
    v = v || {};
    return new THREE.Vector3(v.x || 0, v.y || 0, v.z || 0);
}

function quaternion(q) {
    q = q || {};
    const result = new THREE.Quaternion(q.x || 0, q.y || 0, q.z || 0, q.w || 0);
    // an unset orientation is all zero
    return result.lengthSq() < 1e-9 ? new THREE.Quaternion() : result.normalize();
}

function lineObject(color) {
    const line = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color}));
    line.frustumCulled = false;
    return line;
}

function pointsObject(color, size) {
    const points = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({color, size, sizeAttenuation: true}));
    points.frustumCulled = false;
    return points;
}

function setPositions(object, positions) {
    object.geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    object.geometry.computeBoundingSphere();
}

export class Viewer {
    constructor(container) {
        this.container = container;
        this.renderer = new THREE.WebGLRenderer({antialias: true});
        this.renderer.setPixelRatio(window.devicePixelRatio);
        container.appendChild(this.renderer.domElement);

        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1b1f24);
        this.camera = new THREE.PerspectiveCamera(50, 1, 0.05, 1000);
        this.camera.position.set(-3.5, 3, 3.5);
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.target.set(0, 0.4, 0);

        this.scene.add(new THREE.HemisphereLight(0xffffff, 0x404850, 2.2));
        const sun = new THREE.DirectionalLight(0xffffff, 2.0);
        sun.position.set(5, 10, 6);
        this.scene.add(sun);

        // robot_remote_control (like ROS) uses z up, three.js y up
        this.world = new THREE.Group();
        this.world.rotation.x = -Math.PI / 2;
        this.scene.add(this.world);

        const grid = new THREE.GridHelper(40, 40, 0x56606c, 0x30363e);
        grid.rotation.x = Math.PI / 2;
        this.world.add(grid);
        this.world.add(new THREE.AxesHelper(1));

        this.robotRoot = new THREE.Group();
        this.world.add(this.robotRoot);
        this.robotAxes = new THREE.AxesHelper(0.5);
        this.robotRoot.add(this.robotAxes);
        this.robot = null;

        this.trail = lineObject(0x7d8794);
        this.trailPositions = [];
        this.world.add(this.trail);
        this.path = lineObject(0x4fc3f7);
        this.world.add(this.path);
        this.laser = pointsObject(0xff5a5a, 0.05);
        this.world.add(this.laser);
        this.cloud = pointsObject(0xffb74d, 0.04);
        this.world.add(this.cloud);
        this.maps = new Map();

        this.goalArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.8, 0x66bb6a, 0.25, 0.15);
        this.goalArrow.visible = false;
        this.world.add(this.goalArrow);
        this.waypoints = new THREE.Group();
        this.waypointLine = lineObject(0xba68c8);
        this.world.add(this.waypoints, this.waypointLine);

        this.follow = true;
        this.lastRobotPosition = null;
        this.pickMode = null;
        this.pick = null;
        this.raycaster = new THREE.Raycaster();
        this.setupPicking();

        new ResizeObserver(() => this.resize()).observe(container);
        this.resize();
        this.renderer.setAnimationLoop(() => this.render());
    }

    resize() {
        const width = this.container.clientWidth || 1;
        const height = this.container.clientHeight || 1;
        this.renderer.setSize(width, height);
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
    }

    render() {
        if (this.follow) {
            const position = this.robotRoot.getWorldPosition(new THREE.Vector3());
            if (this.lastRobotPosition) {
                const delta = position.clone().sub(this.lastRobotPosition);
                this.camera.position.add(delta);
                this.controls.target.add(delta);
            }
            this.lastRobotPosition = position;
        } else {
            this.lastRobotPosition = null;
        }
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }

    resetView() {
        const position = this.robotRoot.getWorldPosition(new THREE.Vector3());
        this.controls.target.copy(position).add(new THREE.Vector3(0, 0.4, 0));
        this.camera.position.copy(position).add(new THREE.Vector3(-3.5, 3, 3.5));
        this.lastRobotPosition = null;
    }

    // ------------------------------------------------------------ robot

    /**
     * @param robot URDFRobot
     */
    setRobotModel(robot) {
        if (this.robot) {
            this.robotRoot.remove(this.robot);
        }
        this.robot = robot;
        this.robotRoot.add(robot);
    }

    setPose(pose) {
        this.robotRoot.position.copy(vector(pose.position));
        this.robotRoot.quaternion.copy(quaternion(pose.orientation));
        const last = this.trailPositions.length - 3;
        const position = this.robotRoot.position;
        if (last < 0 || position.distanceToSquared(new THREE.Vector3(...this.trailPositions.slice(last))) > 0.0025) {
            this.trailPositions.push(position.x, position.y, position.z + 0.01);
            if (this.trailPositions.length > MAX_TRAIL_POINTS * 3) {
                this.trailPositions.splice(0, 3);
            }
            setPositions(this.trail, this.trailPositions);
        }
    }

    clearTrail() {
        this.trailPositions = [];
        setPositions(this.trail, []);
    }

    setJointState(state) {
        if (!this.robot) {
            return;
        }
        const names = state.name || [];
        const positions = state.position || [];
        names.forEach((name, i) => {
            if (this.robot.joints[name] && i < positions.length) {
                this.robot.setJointValue(name, positions[i]);
            }
        });
    }

    jointLimits(name) {
        const joint = this.robot && this.robot.joints[name];
        if (!joint) {
            return null;
        }
        if (joint.jointType === 'continuous') {
            return {type: joint.jointType, lower: -Math.PI, upper: Math.PI};
        }
        return {type: joint.jointType, lower: joint.limit.lower, upper: joint.limit.upper};
    }

    /**
     * @returns the object of a frame name used in telemetry headers: a link of the robot or the world.
     * Robots may prefix the link names (e.g. "r1_laser_link" for the link "laser_link").
     */
    frameObject(frame) {
        if (!frame || WORLD_FRAMES.has(frame) || !this.robot) {
            return this.world;
        }
        if (this.robot.frames[frame]) {
            return this.robot.frames[frame];
        }
        const link = Object.keys(this.robot.links)
            .filter((name) => frame.endsWith(name))
            .sort((a, b) => b.length - a.length)[0];
        return link ? this.robot.links[link] : this.world;
    }

    // ------------------------------------------------------------ data

    setPath(poses) {
        const positions = [];
        (poses.poses || []).forEach((pose) => {
            const p = vector(pose.position);
            positions.push(p.x, p.y, p.z + 0.02);
        });
        this.frameObject(poses.header && poses.header.frame).add(this.path);
        setPositions(this.path, positions);
    }

    setLaserScan(scan) {
        const positions = [];
        const ranges = scan.ranges || [];
        const start = scan.startAngle || 0;
        const step = scan.horizontalAngleResolution || 0;
        const min = scan.rangeMin || 0;
        const max = scan.rangeMax || Infinity;
        ranges.forEach((range, i) => {
            if (range > min && range < max) {
                const angle = start + i * step;
                positions.push(range * Math.cos(angle), range * Math.sin(angle), 0);
            }
        });
        this.frameObject(scan.header && scan.header.frame).add(this.laser);
        setPositions(this.laser, positions);
    }

    setPointCloud(cloud) {
        this.updatePointCloud(this.cloud, cloud);
    }

    updatePointCloud(object, cloud) {
        const positions = [];
        (cloud.points || []).forEach((p) => positions.push(p.x || 0, p.y || 0, p.z || 0));
        object.position.copy(vector(cloud.origin && cloud.origin.position));
        object.quaternion.copy(quaternion(cloud.origin && cloud.origin.orientation));
        this.frameObject(cloud.header && cloud.header.frame).add(object);
        setPositions(object, positions);
    }

    /**
     * show a map (the content of a robot_remote_control::Map), supports PointCloud and GridMap
     * @returns a description or throws for unsupported types
     */
    setMap(key, map) {
        this.removeMap(key);
        const type = (map['@type'] || '').split('.').pop();
        let object;
        let description;
        if (type === 'PointCloud') {
            object = pointsObject(0x9ccc65, 0.04);
            this.updatePointCloud(object, map);
            description = `point cloud, ${(map.points || []).length} points`;
        } else if (type === 'GridMap' && map.layers && map.layers.length) {
            object = this.gridMapObject(map);
            description = `grid map ${map.layers[0].name || ''} ${(map.layers[0].size || {}).x || 0}×${(map.layers[0].size || {}).y || 0}`;
        } else {
            throw new Error(`unsupported map type ${map['@type'] || 'unknown'}`);
        }
        this.maps.set(key, object);
        return description;
    }

    gridMapObject(map) {
        const layer = map.layers[0];
        const sizeX = (layer.size && layer.size.x) || 1;
        const sizeY = (layer.size && layer.size.y) || 1;
        const scaleX = (layer.scale && layer.scale.x) || 1;
        const scaleY = (layer.scale && layer.scale.y) || 1;
        const values = layer.value || [];
        let min = Infinity;
        let max = -Infinity;
        values.forEach((v) => {
            if (Number.isFinite(v)) {
                min = Math.min(min, v);
                max = Math.max(max, v);
            }
        });
        const range = max > min ? max - min : 1;
        const data = new Uint8Array(sizeX * sizeY * 4);
        for (let i = 0; i < sizeX * sizeY; ++i) {
            const v = values[i];
            const shade = Number.isFinite(v) ? 40 + 200 * (v - min) / range : 0;
            data.set([shade, shade, shade + 15, Number.isFinite(v) ? 170 : 0], i * 4);
        }
        const texture = new THREE.DataTexture(data, sizeX, sizeY);
        texture.needsUpdate = true;
        texture.magFilter = THREE.NearestFilter;
        const plane = new THREE.Mesh(new THREE.PlaneGeometry(sizeX * scaleX, sizeY * scaleY),
                                     new THREE.MeshBasicMaterial({map: texture, transparent: true, depthWrite: false}));
        // value[x + y * sizeX], the origin is the corner of the grid
        plane.position.set(sizeX * scaleX / 2, sizeY * scaleY / 2, 0.005);
        const group = new THREE.Group();
        group.add(plane);
        group.position.copy(vector(map.origin && map.origin.position));
        group.quaternion.copy(quaternion(map.origin && map.origin.orientation));
        this.frameObject(map.frame || (map.header && map.header.frame)).add(group);
        return group;
    }

    removeMap(key) {
        const object = this.maps.get(key);
        if (object && object.parent) {
            object.parent.remove(object);
        }
        this.maps.delete(key);
    }

    setMapVisible(key, visible) {
        const object = this.maps.get(key);
        if (object) {
            object.visible = visible;
        }
    }

    setLayerVisible(layer, visible) {
        const objects = {robot: this.robotRoot, trail: this.trail, path: this.path, laser: this.laser, cloud: this.cloud};
        if (objects[layer]) {
            objects[layer].visible = visible;
        }
    }

    // ------------------------------------------------------------ picking

    /**
     * @param mode null, 'goal' (drag for the orientation, callback(x, y, yaw or null))
     *             or 'waypoints' (click, callback(x, y))
     */
    setPickMode(mode, callback) {
        this.pickMode = mode;
        this.pickCallback = callback;
        this.goalArrow.visible = false;
        this.renderer.domElement.style.cursor = mode ? 'crosshair' : '';
    }

    setWaypoints(points) {
        this.waypoints.clear();
        const positions = [];
        points.forEach((p) => {
            const marker = new THREE.Mesh(new THREE.SphereGeometry(0.08, 12, 8), new THREE.MeshBasicMaterial({color: 0xba68c8}));
            marker.position.set(p.x, p.y, 0.05);
            this.waypoints.add(marker);
            positions.push(p.x, p.y, 0.05);
        });
        setPositions(this.waypointLine, positions);
    }

    groundPoint(event) {
        const rect = this.renderer.domElement.getBoundingClientRect();
        const pointer = new THREE.Vector2(((event.clientX - rect.left) / rect.width) * 2 - 1,
                                          -((event.clientY - rect.top) / rect.height) * 2 + 1);
        this.raycaster.setFromCamera(pointer, this.camera);
        const hit = this.raycaster.ray.intersectPlane(GROUND, new THREE.Vector3());
        return hit ? this.world.worldToLocal(hit) : null;
    }

    setupPicking() {
        const element = this.renderer.domElement;
        element.addEventListener('pointerdown', (event) => {
            if (!this.pickMode || event.button !== 0) {
                return;
            }
            const point = this.groundPoint(event);
            if (!point) {
                return;
            }
            this.controls.enabled = false;
            element.setPointerCapture(event.pointerId);
            this.pick = {start: point, yaw: null};
            if (this.pickMode === 'goal') {
                this.goalArrow.position.set(point.x, point.y, 0.05);
                this.goalArrow.setDirection(new THREE.Vector3(1, 0, 0));
                this.goalArrow.visible = true;
            }
        });
        element.addEventListener('pointermove', (event) => {
            if (!this.pick || this.pickMode !== 'goal') {
                return;
            }
            const point = this.groundPoint(event);
            if (point && point.distanceTo(this.pick.start) > 0.15) {
                this.pick.yaw = Math.atan2(point.y - this.pick.start.y, point.x - this.pick.start.x);
                this.goalArrow.setDirection(new THREE.Vector3(Math.cos(this.pick.yaw), Math.sin(this.pick.yaw), 0));
            }
        });
        element.addEventListener('pointerup', () => {
            if (!this.pick) {
                return;
            }
            const {start, yaw} = this.pick;
            this.pick = null;
            this.controls.enabled = true;
            if (this.pickCallback) {
                this.pickCallback(start.x, start.y, yaw);
            }
        });
    }
}
