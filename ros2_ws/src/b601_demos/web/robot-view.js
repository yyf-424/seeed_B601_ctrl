import * as THREE from "three";
import { OrbitControls } from "/vendor/OrbitControls.js";
import { STLLoader } from "/vendor/STLLoader.js";

function place(object, frame) {
  object.position.fromArray(frame.xyz);
  object.rotation.set(...frame.rpy, "ZYX");
}

async function visualGeometry(visual, loader) {
  if (visual.kind === "mesh") return loader.loadAsync(visual.url);
  if (visual.kind === "box") return new THREE.BoxGeometry(...visual.size);
  if (visual.kind === "sphere") return new THREE.SphereGeometry(visual.radius, 24, 16);
  if (visual.kind === "cylinder") {
    const body = new THREE.CylinderGeometry(visual.radius, visual.radius, visual.length, 32);
    return body.rotateX(Math.PI / 2);
  }
  throw new Error("不支持的模型几何");
}

class RobotLayer {
  constructor(model, visualGroups, ghostMaterial) {
    this.root = new THREE.Group();
    this.links = new Map();
    for (const [name, visuals] of visualGroups) {
      const link = new THREE.Group();
      for (const { visual, geometry, material } of visuals) {
        const mesh = new THREE.Mesh(geometry, ghostMaterial || material);
        place(mesh, visual);
        if (visual.scale) mesh.scale.fromArray(visual.scale);
        if (ghostMaterial) mesh.renderOrder = 1;
        link.add(mesh);
      }
      this.links.set(name, link);
      this.root.add(link);
    }
    this.joints = [];
    for (const joint of model.joints) {
      const offset = new THREE.Group();
      const motion = new THREE.Group();
      place(offset, joint);
      this.links.get(joint.parent).add(offset);
      offset.add(motion);
      motion.add(this.links.get(joint.child));
      const axis = new THREE.Vector3(...joint.axis).normalize();
      this.joints.push({ ...joint, motion, axis });
    }
  }

  setPose(positions) {
    for (const { name, type, mimic, motion, axis } of this.joints) {
      const position = mimic ? (positions[mimic.joint] ?? 0) * mimic.multiplier + mimic.offset
        : positions[name] ?? 0;
      if (type === "prismatic") motion.position.copy(axis).multiplyScalar(position);
      if (type === "revolute" || type === "continuous") motion.quaternion.setFromAxisAngle(axis, position);
    }
  }

  tipText() {
    const tip = this.links.get("end_link");
    if (!tip) return "—";
    return tip.getWorldPosition(new THREE.Vector3()).toArray()
      .map((value) => value.toFixed(3)).join("  /  ");
  }
}

export function createRobotView(section) {
  const stage = section.querySelector("#robot-stage");
  const notice = section.querySelector("#model-status");
  const actualTip = section.querySelector("#actual-tip");
  const targetTip = section.querySelector("#target-tip");
  const targetLabel = section.querySelector("#model-target-label");
  const showTarget = section.querySelector("#show-robot-target");
  const reset = section.querySelector("#reset-view");
  let actualLayer;
  let targetLayer;
  let latest = { feedback: {}, target: {}, label: "", online: false };
  let loading = false;
  let nextAttempt = 0;
  let requestedFrame = 0;
  let assetText = "";
  let viewportWidth = 0;
  let viewportHeight = 0;

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
  renderer.domElement.setAttribute("role", "img");
  renderer.domElement.setAttribute("aria-label", "B601 实测与目标三维模型");
  stage.prepend(renderer.domElement);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color("#0a1924");
  const camera = new THREE.PerspectiveCamera(38, 1, 0.03, 15);
  camera.up.set(0, 0, 1);
  camera.position.set(1.22, -0.98, 0.86);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.32);
  controls.update();
  controls.saveState();
  controls.addEventListener("change", draw);
  reset.addEventListener("click", () => controls.reset());
  showTarget.addEventListener("change", applyPose);
  const grid = new THREE.GridHelper(1.8, 18, "#3e6275", "#203b4b");
  grid.rotation.x = Math.PI / 2;
  scene.add(grid, new THREE.HemisphereLight(0xe8f4ff, 0x6d7f91, 2));
  const key = new THREE.DirectionalLight(0xffffff, 2.0);
  key.position.set(1.8, -1.2, 2.4);
  const fill = new THREE.DirectionalLight(0x9cdbff, 0.9);
  fill.position.set(-1.5, 1.6, 1.0);
  scene.add(key, fill);

  function draw() {
    if (requestedFrame) return;
    requestedFrame = window.requestAnimationFrame(() => {
      requestedFrame = 0;
      const { clientWidth, clientHeight } = stage;
      if (!clientWidth || !clientHeight) return;
      if (viewportWidth !== clientWidth || viewportHeight !== clientHeight) {
        viewportWidth = clientWidth;
        viewportHeight = clientHeight;
        camera.aspect = clientWidth / clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(clientWidth, clientHeight, false);
      }
      renderer.render(scene, camera);
      if (actualLayer) actualTip.textContent = actualLayer.root.visible ? actualLayer.tipText() : "—";
      if (targetLayer) targetTip.textContent = targetLayer.root.visible ? targetLayer.tipText() : "—";
    });
  }
  new ResizeObserver(draw).observe(stage);

  function applyPose() {
    if (!actualLayer) return;
    const { feedback, target, label, online } = latest;
    const hasFeedback = actualLayer.joints.every(({ type, name, mimic }) =>
      type === "fixed" || mimic || Number.isFinite(feedback[name]));
    actualLayer.root.visible = hasFeedback;
    targetLayer.root.visible = hasFeedback && online && showTarget.checked && Object.keys(target).length > 0;
    if (hasFeedback) actualLayer.setPose(feedback);
    if (targetLayer.root.visible) targetLayer.setPose({ ...feedback, ...target });
    notice.textContent = !hasFeedback ? "等待关节反馈" : online ? assetText : "反馈中断 · 显示最后一帧";
    targetLabel.textContent = targetLayer.root.visible ? label : "关节目标";
    draw();
  }

  async function loadModel() {
    loading = true;
    try {
      const response = await fetch("/api/model");
      if (response.status === 503) return;
      if (!response.ok) throw new Error("模型接口不可用");
      const model = await response.json();
      const loader = new STLLoader();
      const visualGroups = await Promise.all(Object.entries(model.links).map(async ([name, visuals]) =>
        [name, await Promise.all(visuals.map(async (visual) => ({
          visual,
          geometry: await visualGeometry(visual, loader),
          material: new THREE.MeshStandardMaterial({
            color: new THREE.Color(...visual.color).convertSRGBToLinear(), roughness: 0.66,
          }),
        })))],
      ));
      const ghost = new THREE.MeshBasicMaterial({
        color: "#54def8", transparent: true, opacity: 0.28, depthWrite: false,
        polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1,
      });
      actualLayer = new RobotLayer(model, visualGroups);
      targetLayer = new RobotLayer(model, visualGroups, ghost);
      scene.add(actualLayer.root, targetLayer.root);
      assetText = model.asset_source === "vendor" ? "B601 官方外观模型" : "B601 简化外观模型";
      reset.disabled = false;
      showTarget.disabled = false;
      section.dataset.ready = "true";
      applyPose();
    } catch (error) {
      notice.textContent = `三维模型加载失败：${error.message}`;
    } finally {
      loading = false;
      nextAttempt = performance.now() + 1800;
    }
  }

  return {
    update(next) {
      latest = next;
      if (actualLayer) applyPose();
      else if (!loading && performance.now() >= nextAttempt) void loadModel();
    },
  };
}
