import { useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { Canvas, useThree } from '@react-three/fiber';
import { Line, OrbitControls } from '@react-three/drei';
import { GROUND_ID, type Body } from '../types';
import { useModelStore } from '../store/useModelStore';
import { mountQuaternion } from '../math/conventions';
import { DESKTOP_QUERY, useMediaQuery } from '../ui/useMediaQuery';
import { buildSolverModel, bodyPoses, nodeWorldPosition, type BodyPose, type SolverModel } from '../sim/kinematics';
import { frameQ, type Trajectory } from '../sim/useSimulation';
import { TRACE_COLOR } from '../theme';
import { GroundGrid } from './GroundGrid';
import { BodyView } from './BodyView';
import { HingeView } from './HingeView';
import { ActuatorView } from './ActuatorView';
import { ContactPlaneView, ContactSphereView } from './ContactView';

/**
 * The 3D view.
 *
 * The scene always draws *some* pose. When a trajectory exists it draws the scrubbed frame;
 * while one is still being computed it falls back to the initial configuration, which is
 * pure kinematics and costs microseconds. That is what lets a slider drag update the view
 * immediately, with the trajectory catching up behind it.
 */

type Props = {
  trajectory: Trajectory | null;
  frameIndex: number;
};

/**
 * Redraw on demand.
 *
 * The canvas runs `frameloop="demand"` so a phone is not re-rendering a static scene at
 * 60fps. That means anything changing the scene has to ask for a frame: React-driven
 * changes do it through the reconciler, the store subscription covers direct edits, and the
 * token covers scrubbing, which changes no React-visible prop of the canvas itself.
 */
function InvalidateOnChange({ token }: { token: unknown }) {
  const invalidate = useThree((state) => state.invalidate);
  useEffect(() => invalidate(), [invalidate, token]);
  useEffect(() => useModelStore.subscribe(() => invalidate()), [invalidate]);
  return null;
}

/**
 * Point the camera at the model rather than at the world origin.
 *
 * A model hanging below its mount — which is most of them — would otherwise sit in the
 * bottom of the frame with empty sky above it. The target is recomputed only when the *set*
 * of bodies changes, never when a pose does: re-aiming on every slider nudge would yank the
 * view out from under whoever is dragging it, and re-aiming during playback would turn an
 * orbit into a chase camera.
 */
function AutoFrame({
  center,
  radius,
  token,
}: {
  center: THREE.Vector3;
  radius: number;
  token: string;
}) {
  const controls = useThree((state) => state.controls) as
    | { target: THREE.Vector3; update: () => void }
    | null;
  const camera = useThree((state) => state.camera);
  const applied = useRef<string | null>(null);

  useEffect(() => {
    if (!controls || applied.current === token) return;
    applied.current = token;

    // Pull the camera back along whichever direction it is already looking from, so the
    // framing distance adapts to the model's size without throwing away the viewing angle.
    // A model in millimetres and one in kilometres both end up filling the frame.
    //
    // Fitting a sphere of radius R in a 45° vertical field needs R/sin(22.5°) ≈ 2.6R. The
    // margin on top of that is for everything drawn *outside* the nodes the radius was
    // measured from — axis triads, hinge rings, actuator glyphs and labels all stick out.
    // `fov` is the *vertical* field, so a portrait phone sees a much narrower horizontal
    // one. Without this the same model that fits on a desktop overflows the sides of a
    // phone.
    const aspect = 'aspect' in camera ? (camera.aspect as number) : 1;
    const narrowness = aspect > 0 && aspect < 1 ? 1 / aspect : 1;

    const direction = camera.position.clone().sub(controls.target);
    if (direction.lengthSq() < 1e-12) direction.set(3.6, 2.4, 4.6);
    camera.position.copy(center).add(direction.normalize().multiplyScalar(radius * 4.2 * narrowness));

    controls.target.copy(center);
    controls.update();
  }, [controls, camera, center, radius, token]);

  return null;
}

/**
 * The path traced by the selected body's centre of mass over the whole run.
 *
 * Only the selected body: tracing everything turns into spaghetti on a chain of any length,
 * and the trace answers "where did *this* go", which is a question about one body at a time.
 */
function useTrace(
  trajectory: Trajectory | null,
  solver: SolverModel | { error: string },
  bodyId: string,
  bodies: Record<string, Body>,
): [number, number, number][] | null {
  return useMemo(() => {
    if (!trajectory || 'error' in solver || trajectory.count < 2) return null;
    const body = bodies[bodyId];
    if (!body || body.isGround) return null;
    const local = body.nodes[body.comNodeId]?.position ?? [0, 0, 0];

    // Subsampled: a few hundred points is plenty for a readable path, and rebuilding
    // thousands of them on every frame would undo the point of the worker.
    const stride = Math.max(1, Math.ceil(trajectory.count / 400));
    const points: [number, number, number][] = [];
    for (let i = 0; i < trajectory.count; i += stride) {
      const pose = bodyPoses(solver, frameQ(trajectory, i)).get(bodyId);
      if (pose) points.push(nodeWorldPosition(pose, local));
    }
    return points.length > 1 ? points : null;
  }, [trajectory, solver, bodyId, bodies]);
}

export function SceneCanvas({ trajectory, frameIndex }: Props) {
  const bodies = useModelStore((s) => s.bodies);
  const hinges = useModelStore((s) => s.hinges);
  const actuators = useModelStore((s) => s.actuators);
  const contactSpheres = useModelStore((s) => s.contactSpheres);
  const contactPlanes = useModelStore((s) => s.contactPlanes);
  const settings = useModelStore((s) => s.settings);
  const upAxis = useModelStore((s) => s.conventions.upAxis);
  const selectedBodyId = useModelStore((s) => s.selectedBodyId);
  const selectedHingeId = useModelStore((s) => s.selectedHingeId);
  const selectedActuatorId = useModelStore((s) => s.selectedActuatorId);
  const selectBody = useModelStore((s) => s.selectBody);
  const selectHinge = useModelStore((s) => s.selectHinge);
  const selectActuator = useModelStore((s) => s.selectActuator);

  const isDesktop = useMediaQuery(DESKTOP_QUERY);
  const mount = useMemo(() => mountQuaternion(upAxis), [upAxis]);

  const solver = useMemo(
    () => buildSolverModel(bodies, hinges, actuators, settings),
    [bodies, hinges, actuators, settings],
  );

  const poses = useMemo(() => {
    if ('error' in solver) return new Map<string, BodyPose>();
    const q =
      trajectory && trajectory.count > 0
        ? frameQ(trajectory, Math.min(frameIndex, trajectory.count - 1))
        : solver.model.q0;
    return bodyPoses(solver, q);
  }, [solver, trajectory, frameIndex]);

  /**
   * The model's bounding sphere, in three.js space.
   *
   * Measured over **every node**, not over body origins. A body's origin says nothing about
   * how far it reaches — an arm's origin sits at its shoulder while the body extends a metre
   * past it — so framing on origins alone puts the camera inside the model.
   *
   * The model lives under the mount group, so each point is carried through the same
   * rotation before it can be used as a camera target. On a phone the centre drops further
   * so the model clears the sheet.
   */
  const bounds = useMemo(() => {
    const box = new THREE.Box3();
    const point = new THREE.Vector3();
    let any = false;

    for (const [id, pose] of poses) {
      const body = bodies[id];
      if (!body || body.isGround) continue;
      for (const nodeId of body.nodeOrder) {
        const node = body.nodes[nodeId];
        if (!node) continue;
        point.set(...nodeWorldPosition(pose, node.position)).applyQuaternion(mount);
        box.expandByPoint(point);
        any = true;
      }
    }

    const center = any ? box.getCenter(new THREE.Vector3()) : new THREE.Vector3();
    // Half the diagonal, so the sphere contains the box rather than just its half-width.
    const radius = any ? Math.max(box.getSize(new THREE.Vector3()).length() / 2, 1e-3) : 1;
    if (!isDesktop) center.y -= radius * 0.55;
    return { center, radius };
  }, [poses, bodies, mount, isDesktop]);

  // Only the identity of the bodies, so re-aiming happens on add/delete and not on motion.
  const frameToken = useMemo(
    () => `${Object.keys(bodies).sort().join(',')}|${isDesktop}`,
    [bodies, isDesktop],
  );

  const actuatorReference = useMemo(() => {
    let max = 0;
    for (const actuator of Object.values(actuators)) max = Math.max(max, Math.hypot(...actuator.vector));
    return max;
  }, [actuators]);

  const trace = useTrace(trajectory, solver, selectedBodyId, bodies);

  /**
   * Marker size, from the model's own extent, so node dots and labels stay proportionate
   * whether the model is millimetres or kilometres across.
   */
  const scale = Math.max(0.05, bounds.radius);

  const cameraPosition: [number, number, number] = isDesktop ? [3.6, 2.4, 4.6] : [4.6, 3.2, 5.8];

  return (
    <Canvas
      // Capped device pixel ratio: 3× on a modern phone triples the fragment cost for no
      // visible gain on geometry this simple.
      dpr={[1, 2]}
      frameloop="demand"
      camera={{ position: cameraPosition, fov: 45, near: 0.05, far: 400 }}
      gl={{ antialias: true }}
      onPointerMissed={() => selectBody(GROUND_ID)}
    >
      <color attach="background" args={['#0b0e14']} />
      <fog attach="fog" args={['#0b0e14', 22, 64]} />

      <ambientLight intensity={1.15} />
      <directionalLight position={[6, 10, 8]} intensity={1.4} />
      <directionalLight position={[-8, -4, -6]} intensity={0.45} />

      {/* Drawn in three.js space, outside the mount group: drei's Grid lies in the three.js
          XZ plane, which is the horizontal plane on screen whichever engineering axis is up.
          Inside the mount it would tip on its side in Z-up. */}
      <GroundGrid />

      {/* Everything with real coordinates lives under the mount, the single place the
          up-axis convention is applied. */}
      <group quaternion={mount}>
        {trace && (
          <Line points={trace} color={TRACE_COLOR} lineWidth={1.4} transparent opacity={0.7} />
        )}

        {Object.values(bodies).map((body) => {
          if (body.isGround || !body.visible) return null;
          const pose = poses.get(body.id);
          if (!pose) return null;
          return (
            <BodyView
              key={body.id}
              body={body}
              pose={pose}
              selected={body.id === selectedBodyId}
              scale={scale}
              onSelect={() => selectBody(body.id)}
            />
          );
        })}

        {Object.values(hinges).map((hinge) => (
          <HingeView
            key={hinge.id}
            hinge={hinge}
            bodies={bodies}
            poses={poses}
            selected={hinge.id === selectedHingeId}
            scale={scale}
            onSelect={() => selectHinge(hinge.id)}
          />
        ))}

        {Object.values(actuators).map((actuator) => (
          <ActuatorView
            key={actuator.id}
            actuator={actuator}
            bodies={bodies}
            poses={poses}
            selected={actuator.id === selectedActuatorId}
            scale={scale}
            reference={actuatorReference}
            onSelect={() => selectActuator(actuator.id)}
          />
        ))}

        {Object.values(contactSpheres).map((sphere) => {
          const body = bodies[sphere.bodyId];
          const pose = poses.get(sphere.bodyId);
          return body && pose ? <ContactSphereView key={sphere.id} sphere={sphere} body={body} pose={pose} scale={scale} /> : null;
        })}

        {Object.values(contactPlanes).map((plane) => (
          <ContactPlaneView key={plane.id} plane={plane} />
        ))}
      </group>

      <OrbitControls
        makeDefault
        enableDamping
        dampingFactor={0.12}
        minDistance={0.5}
        maxDistance={80}
        maxPolarAngle={Math.PI * 0.495}
      />

      <AutoFrame center={bounds.center} radius={bounds.radius} token={frameToken} />
      <InvalidateOnChange token={frameIndex} />
    </Canvas>
  );
}
