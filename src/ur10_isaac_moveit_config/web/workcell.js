import * as THREE from './workcell-vendor/three.module.js';
import { OrbitControls } from './workcell-vendor/OrbitControls.js';
import { RoomEnvironment } from './workcell-vendor/RoomEnvironment.js';

const viewport = document.getElementById('viewport');
const robotStatus = document.getElementById('robot-state');
const sceneStatus = document.getElementById('scene-state');

function readSTL(buffer) {
  // The bundled JAKA CAD and the exported torch use binary STL, in metres.
  const source = new DataView(buffer);
  const count = source.getUint32(80, true);
  if (84 + count * 50 !== buffer.byteLength) throw new Error('Invalid binary STL');
  const vertices = new Float32Array(count * 9);
  for (let face = 0; face < count; face++) {
    for (let axis = 0; axis < 9; axis++) {
      vertices[face * 9 + axis] = source.getFloat32(96 + face * 50 + axis * 4, true);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
  geometry.computeVertexNormals();
  return geometry;
}

async function createViewer() {
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(38, 1, .01, 25);
  camera.up.set(0, 0, 1);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.75));
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = .9;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  viewport.append(renderer.domElement);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.minDistance = .15;
  controls.maxDistance = 15;

  // Studio lighting and orbit controls adapted from the scanner workspace.
  scene.background = new THREE.Color(0x20272e);
  scene.add(new THREE.HemisphereLight(0xdceaff, 0x59616a, .65));
  const key = new THREE.DirectionalLight(0xfff1dc, 2);
  key.position.set(.85, -1, 1.8);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  Object.assign(key.shadow.camera, {left: -1.8, right: 1.8, top: 1.8, bottom: -1.8, near: .05, far: 6});
  key.shadow.normalBias = .00035;
  const rim = new THREE.DirectionalLight(0xc0daff, 1.1);
  rim.position.set(-.9, .5, 1);
  scene.add(key, rim);
  const room = new RoomEnvironment();
  const pmrem = new THREE.PMREMGenerator(renderer);
  const reflections = pmrem.fromScene(room, .035);
  scene.environment = reflections.texture;
  scene.environmentIntensity = .55;
  room.dispose();
  pmrem.dispose();

  const parts = new THREE.Group();
  const arm = new THREE.Group();
  scene.add(parts, arm);
  arm.visible = false;
  const links = new Map();
  const body = new THREE.MeshStandardMaterial({color: 0xe3e9ed, roughness: .4, metalness: .28});
  const torch = new THREE.MeshStandardMaterial({color: 0x646a71, roughness: .34, metalness: .6});
  const table = new THREE.MeshStandardMaterial({color: 0x34393b, roughness: .7, metalness: .2});
  const frameMaterial = new THREE.MeshStandardMaterial({color: 0xab262c, roughness: .65});
  const holeMaterial = new THREE.MeshBasicMaterial({color: 0x171a1d});
  const part = new THREE.MeshStandardMaterial({color: 0xc48753, roughness: .43, metalness: .52});

  async function attach(frame, name, material) {
    const response = await fetch('/workcell-mesh/' + name + '.stl');
    if (!response.ok) throw new Error(name);
    const mesh = new THREE.Mesh(readSTL(await response.arrayBuffer()), material);
    mesh.castShadow = mesh.receiveShadow = true;
    links.get(frame).add(mesh);
  }

  const frames = Array.from({length: 7}, (_, i) => 'Link_' + i).concat('welding_torch_link');
  for (const frame of frames) {
    const link = new THREE.Group();
    links.set(frame, link);
    arm.add(link);
  }
  let meshesReady = false;
  let meshesFailed = false;
  let meshesRequested = false;
  function loadArm() {
    if (meshesRequested) return;
    meshesRequested = true;
    const promises = frames.slice(0, 7).map((frame, i) => attach(frame, i || 'base_link', body));
    for (let i = 0; i < 11; i++) promises.push(attach('welding_torch_link', 'visual_' + i, torch));
    Promise.all(promises).then(() => { meshesReady = true; }, () => { meshesFailed = true; });
  }

  function applyPose(mesh, pose) {
    mesh.position.fromArray(pose.position);
    mesh.quaternion.fromArray(pose.orientation).normalize();
  }

  function buildTable(object) {
    const [width, depth, collisionThickness] = object.size;
    const group = new THREE.Group();
    const top = collisionThickness / 2;
    const height = .85;
    const thickness = Math.min(.05, collisionThickness);
    function box(size, position, material) {
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), material);
      mesh.position.fromArray(position);
      mesh.castShadow = mesh.receiveShadow = true;
      group.add(mesh);
    }
    box([width, depth, thickness], [0, 0, top - thickness / 2], table);
    const xPosts = [-width / 2 + .045, width / 2 - .045];
    const yPosts = [-depth / 2 + .045, depth / 2 - .045];
    for (const x of xPosts) for (const y of yPosts) {
      box([.04, .04, height - thickness], [x, y, top - (height + thickness) / 2], frameMaterial);
    }
    for (const level of [.06, .33]) {
      const z = top - height + level;
      for (const y of yPosts) box([width - .05, .025, .03], [0, y, z], frameMaterial);
      for (const x of xPosts) box([.025, depth - .05, .03], [x, 0, z], frameMaterial);
    }
    if (object.perforations?.centers.length) {
      const {diameter, centers} = object.perforations;
      const holes = new THREE.InstancedMesh(new THREE.CircleGeometry(diameter / 2, 16),
                                           holeMaterial, centers.length);
      const matrix = new THREE.Matrix4();
      centers.forEach(([x, y], index) => {
        holes.setMatrixAt(index, matrix.makeTranslation(x, y, top + .0002));
      });
      group.add(holes);
    }
    return group;
  }

  function updateParts(objects) {
    for (const item of [...parts.children]) {
      item.traverse(mesh => mesh.geometry?.dispose());
      parts.remove(item);
    }
    for (const object of objects) {
      let mesh;
      if (object.id === 'floor') {
        mesh = buildTable(object);
      } else {
        let geometry;
        if (object.primitive === 'box') {
          geometry = new THREE.BoxGeometry(...object.size);
        } else if (object.primitive === 'cylinder') {
          const [height, radius] = object.size;
          geometry = new THREE.CylinderGeometry(radius, radius, height, 32);
          geometry.rotateX(Math.PI / 2);
        } else {
          geometry = new THREE.BufferGeometry();
          geometry.setAttribute('position', new THREE.Float32BufferAttribute(object.vertices, 3));
          geometry.setIndex(object.indices);
          geometry.computeVertexNormals();
        }
        mesh = new THREE.Mesh(geometry, part);
        mesh.castShadow = mesh.receiveShadow = true;
      }
      applyPose(mesh, object);
      parts.add(mesh);
    }
  }

  function frame(top = false) {
    const bounds = new THREE.Box3().setFromObject(parts);
    if (arm.visible) bounds.expandByObject(arm);
    const target = bounds.isEmpty() ? new THREE.Vector3(0, -.35, .24) : bounds.getCenter(new THREE.Vector3());
    const size = bounds.isEmpty() ? 1.2 : Math.max(1, bounds.getSize(new THREE.Vector3()).length());
    controls.target.copy(target);
    const direction = top ? new THREE.Vector3(0, -.001, 1) : new THREE.Vector3(1.05, -1.4, 1);
    camera.position.copy(target).addScaledVector(direction.normalize(), size * 1.65);
    controls.update();
  }
  frame();
  document.getElementById('fit').onclick = () => frame();
  document.getElementById('top').onclick = () => frame(true);
  function resize() {
    camera.aspect = viewport.clientWidth / Math.max(1, viewport.clientHeight);
    camera.updateProjectionMatrix();
    renderer.setSize(viewport.clientWidth, viewport.clientHeight, false);
  }
  resize();
  new ResizeObserver(resize).observe(viewport);
  renderer.setAnimationLoop(() => {
    controls.update();
    renderer.render(scene, camera);
  });

  let revision = -1;
  let fitted = false;
  let real = null;
  let measuredPose = null;
  let lastAnimatedPose = null;

  function readPose(points) {
    const poses = new Map((points || []).map(pose => [pose.frame, pose]));
    return frames.slice(0, 7).every(frame => poses.has(frame)) ? poses : null;
  }
  function showPose(poses) {
    for (const [frame, link] of links) {
      const pose = poses.get(frame);
      link.visible = Boolean(pose);
      if (pose) applyPose(link, pose);
    }
    arm.visible = true;
  }
  function showMissingPose(reason) {
    const fallback = measuredPose || lastAnimatedPose;
    if (meshesReady && fallback) showPose(fallback);
    robotStatus.classList.toggle('stale', arm.visible);
    const label = measuredPose && real ? 'Последняя измеренная поза' : 'Последняя расчётная поза';
    robotStatus.textContent = arm.visible ? label + ' · ' + reason : reason;
  }
  async function poll() {
    try {
      const response = await fetch('/api/workcell?revision=' + revision, {
        cache: 'no-store', signal: AbortSignal.timeout(2500)
      });
      if (!response.ok) throw new Error('Workcell unavailable');
      const data = await response.json();
      if (data.objects) {
        updateParts(data.objects);
        revision = data.revision;
      }
      if (real !== data.real) {
        real = data.real;
        arm.visible = false;
        measuredPose = lastAnimatedPose = null;
      }
      loadArm();
      const robot = data.robot || {};
      const wasVisible = arm.visible;
      const freshPose = robot.available ? readPose(robot.points) : null;
      measuredPose = freshPose || readPose(data.last_pose?.points) || measuredPose;
      const animated = real ? readPose(data.animation) : null;
      if (meshesReady && animated) {
        lastAnimatedPose = animated;
        showPose(animated);
        robotStatus.textContent = 'Анимация траектории MoveIt · расчётная поза';
        robotStatus.classList.add('stale');
      } else if (meshesReady && freshPose) {
        showPose(freshPose);
        robotStatus.textContent = real ? 'Положение JAKA ZU12 из ROS' : 'JAKA ZU12 · поза по суставам UR10';
        robotStatus.classList.remove('stale');
      } else {
        showMissingPose(meshesFailed ? 'Модель робота недоступна' :
          !meshesReady ? 'Загружаем модель' : robot.reason || 'Нет свежих данных TF');
      }
      sceneStatus.textContent = data.live ? 'Данные MoveIt' : 'Нет свежих данных MoveIt';
      const firstArmPose = arm.visible && !wasVisible;
      const canFit = arm.visible || meshesFailed || !robot.available;
      if (data.live && parts.children.length && canFit && (!fitted || firstArmPose)) {
        fitted = true;
        frame();
      }
    } catch {
      showMissingPose('Нет связи с панелью');
      sceneStatus.textContent = 'Нет свежих данных MoveIt';
    } finally {
      setTimeout(poll, 220);
    }
  }
  poll();
}

createViewer().catch(error => {
  console.error(error);
  sceneStatus.textContent = 'WebGL недоступен';
});
