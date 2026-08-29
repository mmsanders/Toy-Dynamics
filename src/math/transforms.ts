import * as THREE from 'three';
import type { DofSpec, Quat, Vec3 } from '../types';

/**
 * Local frame composition, for the store and the scene.
 *
 * The solver has its own spatial-vector machinery in `src/dyn`, tuned for running inside an
 * integrator. This module is the other kind: a handful of readable pose operations used
 * when editing the model and when drawing it, where clarity matters more than avoiding an
 * allocation. All rotation maths delegates to three.js rather than being hand-rolled.
 */

export type Pose = {
  /** Origin, in the parent frame. */
  position: Vec3;
  /** Rotation from the parent frame's axes to this frame's axes. */
  orientation: Quat;
};

export const IDENTITY_POSE: Pose = { position: [0, 0, 0], orientation: [0, 0, 0, 1] };

export const toThreeQuat = (q: Quat): THREE.Quaternion =>
  new THREE.Quaternion(q[0], q[1], q[2], q[3]);

export const toThreeVec = (v: Vec3): THREE.Vector3 => new THREE.Vector3(v[0], v[1], v[2]);

export const fromThreeQuat = (q: THREE.Quaternion): Quat => [q.x, q.y, q.z, q.w];

export const fromThreeVec = (v: THREE.Vector3): Vec3 => [v.x, v.y, v.z];

/** Rotate a vector by a quaternion. */
export function applyQuat(v: Vec3, q: Quat): Vec3 {
  return fromThreeVec(toThreeVec(v).applyQuaternion(toThreeQuat(q)));
}

/**
 * Compose two poses: `b` expressed in `a`'s frame, giving `b` in `a`'s parent frame.
 *
 * The position accumulates through `a`'s rotation, which is the step that gets dropped in
 * hand-written versions and quietly leaves child frames in the wrong place.
 */
export function composePose(a: Pose, b: Pose): Pose {
  const qa = toThreeQuat(a.orientation);
  const position = toThreeVec(b.position).applyQuaternion(qa).add(toThreeVec(a.position));
  const orientation = qa.clone().multiply(toThreeQuat(b.orientation));
  return { position: fromThreeVec(position), orientation: fromThreeQuat(orientation) };
}

/** The pose that undoes `p`: if `p` places B in A, this places A in B. */
export function invertPose(p: Pose): Pose {
  const inverse = toThreeQuat(p.orientation).invert();
  const position = toThreeVec(p.position).negate().applyQuaternion(inverse);
  return { position: fromThreeVec(position), orientation: fromThreeQuat(inverse) };
}

/** `b` as seen from `a`, both given in the same parent frame. */
export function relativePose(a: Pose, b: Pose): Pose {
  return composePose(invertPose(a), b);
}

/**
 * The joint's own transform at a given configuration.
 *
 * Mirrors the solver's convention exactly — **translate along the joint axes, then rotate**,
 * with rotations composing as an intrinsic X→Y→Z sequence. Two implementations of one
 * convention is a risk, so this is covered by a test that checks it against `jcalc`.
 */
export function jointPose(dof: readonly DofSpec[], values?: readonly number[]): Pose {
  const value = (i: number): number => values?.[i] ?? dof[i]?.q0 ?? 0;
  const euler = new THREE.Euler(value(3), value(4), value(5), 'XYZ');
  return {
    position: [value(0), value(1), value(2)],
    orientation: fromThreeQuat(new THREE.Quaternion().setFromEuler(euler)),
  };
}

/**
 * Resolve a chain of poses into one.
 *
 * Reads left to right: the first pose is expressed in the outermost frame, each subsequent
 * one in the frame the previous established.
 */
export function chainPoses(poses: Pose[]): Pose {
  return poses.reduce(composePose, IDENTITY_POSE);
}
