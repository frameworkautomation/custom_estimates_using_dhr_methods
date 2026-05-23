import React, { useMemo } from 'react'
import * as THREE from 'three'
import { Line } from '@react-three/drei'

// Map robot move_type → sphere color (unselected)
const MOVE_COLORS = { MoveJ: '#4488ff', MoveL: '#44bb44' }

// Convert ZYX Euler (degrees) to the tool Z-axis direction in scene coords.
// Robot coord system: Z=up. Scene coord system: Y=up (we map robot z→scene y).
// Column 2 of R = Rz*Ry*Rx in robot space, then remap z→y.
function toolZAxis(rxDeg, ryDeg, rzDeg) {
  const toRad = (d) => (d * Math.PI) / 180
  const rx = toRad(rxDeg ?? 0)
  const ry = toRad(ryDeg ?? 0)
  const rz = toRad(rzDeg ?? 0)
  const cx = Math.cos(rx), sx = Math.sin(rx)
  const cy = Math.cos(ry), sy = Math.sin(ry)
  const cz = Math.cos(rz), sz = Math.sin(rz)
  // Column 2 (Z-axis of frame, robot space): [cx*cz*sy+sx*sz, cx*sy*sz-cz*sx, cx*cy]
  const robotX = cx * cz * sy + sx * sz
  const robotY = cx * sy * sz - cz * sx
  const robotZ = cx * cy
  // Map robot→scene: [x, z, y]
  return new THREE.Vector3(robotX, robotZ, robotY).normalize()
}

const SPHERE_R = 25   // mm — visible at scene scale
const ARROW_LEN = 120 // mm

export default function Waypoint({ waypoint: wp, isSelected, onSelect }) {
  const pos = [wp.x ?? 0, wp.z ?? 0, wp.y ?? 0]
  const color = isSelected ? '#ffff00' : (MOVE_COLORS[wp.move_type] ?? '#aaaaaa')

  const arrowDir = useMemo(
    () => toolZAxis(wp.rx, wp.ry, wp.rz),
    [wp.rx, wp.ry, wp.rz]
  )

  const arrowEnd = [
    pos[0] + arrowDir.x * ARROW_LEN,
    pos[1] + arrowDir.y * ARROW_LEN,
    pos[2] + arrowDir.z * ARROW_LEN,
  ]

  return (
    <group>
      <mesh
        position={pos}
        onClick={(e) => { e.stopPropagation(); onSelect(wp.name) }}
      >
        <sphereGeometry args={[SPHERE_R, 16, 16]} />
        <meshStandardMaterial color={color} />
      </mesh>
      <Line points={[pos, arrowEnd]} color={color} lineWidth={2} />
    </group>
  )
}
