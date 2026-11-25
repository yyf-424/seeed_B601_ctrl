import { MathUtils, Matrix3, Quaternion, Vector3 } from "three";

// Damped differential IK for the TCP position. The existing joint bounds
// apply to every candidate; the numerical step and residual only guide solving.
export function solveTcpPosition(layer, goal, seed, names, limits) {
  const joints = layer.joints.filter((joint) => names.includes(joint.name) && !joint.mimic
    && (joint.type === "revolute" || joint.type === "continuous") && limits[joint.name]);
  if (!joints.length) return null;
  const pose = { ...seed };
  for (const { name } of joints) {
    const { min, max } = limits[name];
    if (!Number.isFinite(pose[name])) return null;
    pose[name] = MathUtils.clamp(pose[name], min, max);
  }

  const damping = 0.022 ** 2;
  const inverse = new Matrix3();
  const pivot = new Vector3();
  const orientation = new Quaternion();
  const error = new Vector3();
  const correction = new Vector3();
  const columns = joints.map(() => new Vector3());
  for (let iteration = 0; iteration < 48; iteration++) {
    layer.setPose(pose);
    layer.root.updateMatrixWorld(true);
    const tip = layer.tipPosition();
    if (!tip) return null;
    error.subVectors(goal, tip);
    if (error.length() <= 0.0015) {
      return Object.fromEntries(joints.map(({ name }) => [name, pose[name]]));
    }
    let xx = damping, yy = damping, zz = damping;
    let xy = 0, xz = 0, yz = 0;
    for (const [index, joint] of joints.entries()) {
      joint.offset.getWorldPosition(pivot);
      joint.offset.getWorldQuaternion(orientation);
      pivot.subVectors(tip, pivot);
      const column = columns[index].copy(joint.axis).applyQuaternion(orientation).cross(pivot);
      const { x, y, z } = column;
      xx += x * x;
      yy += y * y;
      zz += z * z;
      xy += x * y;
      xz += x * z;
      yz += y * z;
    }
    inverse.set(xx, xy, xz, xy, yy, yz, xz, yz, zz).invert();
    correction.copy(error).applyMatrix3(inverse);
    let changed = false;
    for (const [index, { name }] of joints.entries()) {
      const { min, max } = limits[name];
      const step = MathUtils.clamp(columns[index].dot(correction), -0.16, 0.16);
      const next = MathUtils.clamp(pose[name] + step, min, max);
      if (Math.abs(next - pose[name]) > 1e-8) changed = true;
      pose[name] = next;
    }
    if (!changed) return null;
  }
  return null;
}
